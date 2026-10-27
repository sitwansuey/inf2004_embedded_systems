"""Verify a local line-trace UF2 against its ARM ELF and reviewed sources."""
from pathlib import Path
import argparse,hashlib,json,struct
p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();repo=a.repo.resolve(strict=True);out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
verified=json.loads((repo/'tools/robot_bench/source_verification.json').read_text(encoding='utf-8-sig'))
assert all(digest(repo/n)==h.lower() for n,h in verified['files'].items()),'Reviewed source changed'
assert not (repo/'app_program/robot_config.local.h').exists(),'Unexpected local override'
stem='mtk3pico_smp1_usb_cdc_wifi_enc_tmo150_irlineonly50course'
uf2=repo/'build_make'/(stem+'.uf2');elf_path=repo/'build_make'/(stem+'.elf')
data=uf2.read_bytes();assert data and len(data)%512==0
payload={};flat=bytearray()
for i in range(len(data)//512):
    block=data[i*512:(i+1)*512]
    magic0,magic1,flags,address,size,index,count,family=struct.unpack_from('<8I',block)
    assert (magic0,magic1,flags,size,index,count,family)==(0x0A324655,0x9E5D5157,0x2000,256,i,len(data)//512,0xE48BFF56)
    assert struct.unpack_from('<I',block,508)[0]==0x0AB16F30
    assert address==0x10000000+i*256 and address+256<=0x10200000
    for j,value in enumerate(block[32:288]):payload[address+j]=value
    flat.extend(block[32:288])
for marker in (b'IR LINE ONLY BENCH v5: boot STOPPED; ultrasonic DISABLED; supervised USB start only',
 b'IRLINE limits: PWM<=50%% run<=20000ms heartbeat<250ms motor-watchdog<=150ms progress<=750ms',
 b'IRLINE no obstacle protection; white cannot prove line presence; no navigation',
 b'IRLINE course: cross=straight recent-side=hold bounded=500/800ms',b'[IRLINE READY] v=5',b'[IRLINE START]',b'[S3B]'):
    assert marker in flat,marker
assert b'[IRSCAN START]' not in flat and b'[IRSCAN DECODE]' not in flat,'Barcode controller present'
assert b'MOTOR LOCK ON' not in flat,'Wrong locked image'
elf=elf_path.read_bytes();assert elf[:6]==b'\x7fELF\x01\x01'
phoff=struct.unpack_from('<I',elf,28)[0];entsize,count=struct.unpack_from('<HH',elf,42)
compared=0
for i in range(count):
    typ,offset,vaddr,paddr,filesz,memsz,flags,align=struct.unpack_from('<8I',elf,phoff+i*entsize)
    if typ==1 and 0x10000000<=paddr<0x10200000:
        assert all(payload.get(paddr+j)==value for j,value in enumerate(elf[offset:offset+filesz]))
        compared+=filesz
shoff=struct.unpack_from('<I',elf,32)[0];shsize,shcount=struct.unpack_from('<HH',elf,46)
text_bytes=data_bytes=bss_bytes=0
for i in range(shcount):
    name,typ,flags,addr,offset,size,link,info,align,entsize=struct.unpack_from('<10I',elf,shoff+i*shsize)
    if flags&2:
        if typ==8:bss_bytes+=size
        elif flags&1:data_bytes+=size
        else:text_bytes+=size
artifact=out/'CSC02_LINE_COURSE_V5_50PCT_USB.uf2';artifact.write_bytes(data)
record={'profile':'IRLineOnly','source_reviewed':True,'firmware_sha256':digest(artifact),
 'artifact':str(artifact),'uf2_bytes':len(data),'elf_flash_bytes_compared':compared,
 'text':text_bytes,'data':data_bytes,'bss':bss_bytes,'static_ram':data_bytes+bss_bytes,
 'telemetry_bytes':50,'motor_lock':False,'boot_stopped':True,'max_power_percent':50,
 'max_run_ms':20000,'USB_lease_ms':250,'motor_watchdog_ms':150,
 'line_profile_version':5,'course_enabled':True,'cross_route':'STRAIGHT','cross_max_ms':500,'turn_max_ms':800,'ultrasonic_enabled':False,'obstacle_protection':False,'encoder_progress_timeout_ms':750,'barcode_navigation':False,'centre_mask':0,'white_hold_ms':100,'both_white':'ASSUMED_CENTRE_FORWARD','both_black':'STOP_AMBIGUOUS','automatic_line_loss_detection':False,'sample_period_ms':10,'flashed':False,'boot_verified':False,'physical_direction_verified':False,
 'physical_line_following_validated':False,'calibration_changed':False}
(out/'LINE_ONLY_BUILD_RESULT.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
