"""Run the integration Makefile recipes with a native compiler on any host."""
import argparse, os, re, shlex, shutil, subprocess, sys
from pathlib import Path
HERE=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler",default=os.getenv("CC","gcc"))
    parser.add_argument("--compile-only",action="store_true",help="Compile every test without executing binaries")
    args=parser.parse_args()
    compiler=shutil.which(args.compiler)
    if not compiler: parser.error("Native C compiler not found")
    variables={"CC":compiler,"LDFLAGS":""}
    lines=(HERE/"Makefile").read_text().splitlines()
    for line in lines:
        match=re.match(r"(\w+)\s*(?:\?=|=)\s*(.*)",line)
        if match and match[1]!="CC":variables[match[1]]=match[2]
    def expand(s):
        for _ in range(10):
            nxt=re.sub(r"\$\((\w+)\)",lambda m:variables.get(m[1],""),s)
            if nxt==s:return s
            s=nxt
        raise ValueError("Recursive Make variable")
    active=False
    (HERE/".build").mkdir(exist_ok=True)
    for line in lines:
        if line=="test:":active=True;continue
        if active and line and not line[0].isspace():break
        if not active or not line.startswith("\t"):continue
        command=line.strip()
        if args.compile_only and not command.startswith("$(CC)"):continue
        if command.startswith("mkdir"):continue
        if command.startswith("$(CC)"):
            tokens=[compiler]+shlex.split(expand(command[len("$(CC)"):]))
            if os.name=="nt":tokens[-1]+=".exe"
        elif command.startswith("python3 "):tokens=[sys.executable,"-B"]+shlex.split(command)[1:]
        else:
            tokens=shlex.split(command)
            if os.name=="nt":tokens[0]+=".exe"
            tokens[0]=str(HERE/tokens[0])
        print("CHECK",command,flush=True)
        subprocess.run(tokens,cwd=HERE,check=True)
    print("Integration compilation PASS (tests NOT executed)" if args.compile_only else "Integration recipes PASS")
if __name__=="__main__":main()
