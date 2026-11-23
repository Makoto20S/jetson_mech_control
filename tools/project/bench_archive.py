"""Bounded, explicit per-run transfer; never scans historical data."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tarfile

MAX_BYTES = 64 * 1024 * 1024
CHUNK = 48 * 1024

def digest(data):
    return hashlib.sha256(data).hexdigest()

def checked_path(root, name=""):
    root=Path(root).absolute()
    if name:
        p=PurePosixPath(name)
        if p.is_absolute() or "\\" in name or ":" in name or any(x in ("","..",".") for x in name.split("/")):
            raise ValueError("Invalid archive path")
    path=root/name
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError("Archive escapes root")
    for p in [path,*path.parents]:
        if p.exists() or p.is_symlink():
            st=p.lstat()
            if stat.S_ISLNK(st.st_mode) or getattr(st,"st_file_attributes",0)&0x400:
                raise ValueError("Archive links are not allowed")
    return path

def inventory(root):
    checked_path(root)
    files={}
    total=0
    for p in sorted(Path(root).rglob("*")):
        checked_path(root,p.relative_to(root).as_posix())
        if p.is_file():
            total+=p.stat().st_size
            if total>MAX_BYTES:raise ValueError("Run exceeds archive limit; retain remote copy")
            data=p.read_bytes()
            files[p.relative_to(root).as_posix()]={"sha256":digest(data),"size":len(data)}
    return files

def send_archive(root, emit):
    files=inventory(root)
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode="w:gz") as tar:
        for name,record in files.items():
            data=checked_path(root,name).read_bytes()
            if digest(data)!=record["sha256"]:raise ValueError("Run changed while archiving")
            info=tarfile.TarInfo(name);info.size=len(data)
            tar.addfile(info,io.BytesIO(data))
    data=buffer.getvalue()
    if len(data)>MAX_BYTES:raise ValueError("Archive exceeds limit")
    sha=digest(data)
    emit({"type":"archive_begin","sha256":sha,"size":len(data),"files":files})
    for offset in range(0,len(data),CHUNK):
        emit({"type":"archive_chunk","offset":offset,"data":base64.b64encode(data[offset:offset+CHUNK]).decode()})
    emit({"type":"archive_end","sha256":sha})
    return sha,files

def cleanup_acknowledged(root, allowed_parent, files, acknowledged):
    root=Path(root).absolute()
    parent=checked_path(allowed_parent)
    checked_path(root)
    if root.parent!=parent or not root.name.startswith("ref01_bench_"):
        raise ValueError("Not an owned bench run")
    if not acknowledged:return False
    if inventory(root)!=files:raise ValueError("Run changed after transfer; not removed")
    # Exact new run, no links, hashes matched the acknowledged local copy.
    shutil.rmtree(root)
    return True

class ArchiveReceiver:
    def __init__(self, root):
        self.root=Path(root);self.meta=None;self.buffer=None
    def accept(self,event):
        kind=event["type"]
        if kind=="archive_begin":
            if self.meta is not None:raise ValueError("Duplicate archive")
            size=event["size"]
            if type(size) is not int or not 0<size<=MAX_BYTES:raise ValueError("Archive size")
            files=event["files"]
            if not isinstance(files,dict) or len(files)>10000:raise ValueError("Archive manifest")
            total=0
            for name,r in files.items():
                checked_path(self.root,name)
                if name=="archive_verification.json":raise ValueError("Reserved receipt path")
                if type(r["size"]) is not int or r["size"]<0:raise ValueError("File size")
                total+=r["size"]
            if total>MAX_BYTES:raise ValueError("Expanded archive size")
            self.meta=event;self.buffer=io.BytesIO()
        elif kind=="archive_chunk":
            if self.meta is None or event["offset"]!=self.buffer.tell():raise ValueError("Archive sequence")
            data=base64.b64decode(event["data"],validate=True)
            if len(data)>CHUNK or self.buffer.tell()+len(data)>self.meta["size"]:raise ValueError("Archive overflow")
            self.buffer.write(data)
        elif kind=="archive_end":
            if self.meta is None:raise ValueError("No archive")
            data=self.buffer.getvalue()
            sha=digest(data)
            if len(data)!=self.meta["size"] or sha!=self.meta["sha256"] or sha!=event["sha256"]:
                raise ValueError("Archive checksum mismatch")
            if self.root.exists():raise ValueError("Archive destination already exists")
            self.root.mkdir(parents=True)
            seen=set()
            with tarfile.open(fileobj=io.BytesIO(data),mode="r:gz") as tar:
                for member in tar:
                    if not member.isfile() or member.name in seen or member.name not in self.meta["files"]:
                        raise ValueError("Unexpected archive member")
                    r=self.meta["files"][member.name]
                    if member.size!=r["size"]:raise ValueError("Member size mismatch")
                    path=checked_path(self.root,member.name);path.parent.mkdir(parents=True,exist_ok=True)
                    content=tar.extractfile(member).read()
                    if digest(content)!=r["sha256"]:raise ValueError("Member checksum mismatch")
                    with path.open("xb") as output:
                        output.write(content);output.flush();os.fsync(output.fileno())
                    seen.add(member.name)
            if seen!=set(self.meta["files"]) or inventory(self.root)!=self.meta["files"]:
                raise ValueError("Incomplete archive")
            receipt=checked_path(self.root,"archive_verification.json")
            with receipt.open("x",encoding="utf-8") as output:
                json.dump({"sha256":sha,"files":self.meta["files"],"verified":True},output,indent=2)
                output.flush();os.fsync(output.fileno())
            return sha
        else:raise ValueError("Unknown archive event")
        return None
