"""Record the robot USB console. Default is read-only; this never flashes.

Use --arm only with the robot safely supported and confirmed calibration.
ARM renews a 500 ms lease; STOP is sent on exit. Reconnection is not automatic.
"""
import argparse
import time
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',required=True)
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--log',type=Path,required=True)
    parser.add_argument('--arm',action='store_true',help='Explicitly request and renew line tracking at 30 percent')
    parser.add_argument('--reset',action='store_true',help='Explicitly clear a latched fault before the run')
    args=parser.parse_args()
    if not 0<args.seconds<=120:parser.error('--seconds must be in (0,120]')
    import serial
    args.log.parent.mkdir(parents=True,exist_ok=True)
    with serial.Serial(args.port,115200,timeout=0.1,write_timeout=1) as device,args.log.open('w',encoding='utf-8') as log:
        device.dtr=True
        if args.reset:device.write(b'r')
        start=time.monotonic();next_arm=start
        try:
            while time.monotonic()-start<args.seconds:
                if args.arm and time.monotonic()>=next_arm:
                    device.write(b'a');next_arm=time.monotonic()+0.1
                data=device.read(4096)
                if data:
                    line=f'{time.monotonic()-start:.3f} '+data.decode('utf-8',errors='replace')
                    print(line,end='',flush=True);log.write(line);log.flush()
        finally:
            if args.arm or args.reset:device.write(b's');device.flush()

if __name__=='__main__':main()
