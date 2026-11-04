#!/usr/bin/env python3
"""Rebuild/check this fork's H7 application from its vendored source. Never flashes.

Uses the compiler version already pinned by the repository's uv.lock. Retains
the existing debug bootstub/signing convention and never rebuilds the bootstub.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
from importlib.metadata import distribution

from Crypto.PublicKey import RSA

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "panda/board"
SIGNED = BOARD / "obj/panda_h7.bin.signed"
MANIFEST = ROOT / "docs/EPS_48A2_FIRMWARE.json"
TOOLCHAIN_VERSION = "13.2.1.post103"
VERSION = "DEV-EPS48A2-DEBUG"


def digest(data):
  return hashlib.sha256(data).hexdigest()


def wire_hash(path):
  return int.from_bytes(hashlib.sha256(path.read_bytes().replace(b"\r", b"")).digest()[:4], "little")


def source_digest():
  files = []
  for directory in (BOARD, ROOT / "opendbc_repo/opendbc/safety"):
    files.extend(p for p in directory.rglob("*") if p.suffix in (".c", ".h", ".s", ".ld") and
                 "obj" not in p.relative_to(directory).parts and "tests" not in p.relative_to(directory).parts)
  files += [BOARD / "obj/gitversion.h", BOARD / "obj/cert.h", Path(__file__).resolve(),
            BOARD / "crypto/sign.py", BOARD / "crypto/certs/debug", BOARD / "crypto/certs/debug.pub"]
  h = hashlib.sha256()
  for path in sorted(files):
    h.update(path.relative_to(ROOT).as_posix().encode() + b"\0")
    h.update(hashlib.sha256(path.read_bytes().replace(b"\r", b"")).digest())
  return h.hexdigest()


def verify_signature(data):
  key = RSA.import_key((BOARD / "crypto/certs/debug.pub").read_bytes())
  unsigned, signature = data[:-128], data[-128:]
  expected = b"\x00\x01" + b"\xff" * 0x69 + b"\x00" + hashlib.sha1(unsigned[4:]).digest()
  actual = pow(int.from_bytes(signature, "big"), key.e, key.n).to_bytes(128, "big")
  if actual != expected or struct.unpack_from("<I", data)[0] != len(unsigned):
    raise ValueError("invalid H7 application length or debug signature")
  reset = struct.unpack_from("<I", data, 4)[0]
  if not 0x08020000 <= reset < 0x08100000 or not reset & 1:
    raise ValueError("application reset vector is outside the H7 application region")


def build():
  dist = distribution("comma-deps-gcc-arm-none-eabi")
  if dist.version != TOOLCHAIN_VERSION:
    raise RuntimeError(f"install comma-deps-gcc-arm-none-eabi=={TOOLCHAIN_VERSION}")
  compiler = next(Path(dist.locate_file(f)) for f in dist.files
                  if str(f).endswith("toolchain/bin/arm-none-eabi-gcc"))
  objcopy = compiler.with_name("arm-none-eabi-objcopy")
  header = f'#pragma once\n\nextern const uint8_t gitversion[18];\nconst uint8_t gitversion[18] = "{VERSION}";\n'
  (BOARD / "obj/gitversion.h").write_text(header)
  (BOARD / "obj/version").write_text(VERSION + "\n")
  flags = ["-mcpu=cortex-m7", "-mfloat-abi=hard", "-mfpu=fpv5-d16", "-mlittle-endian", "-mthumb",
           "-DSTM32H7", "-DSTM32H725xx", "-DALLOW_DEBUG", "-nostdlib", "-fno-builtin", "-std=gnu11",
           "-fsingle-precision-constant", "-Os", "-g", "-Wall", "-Wextra", "-Wstrict-prototypes", "-Werror",
           "-Ipanda", "-Ipanda/board/stm32h7/inc", "-Iopendbc_repo",
           f"-DHEALTH_PACKET_VERSION=0x{wire_hash(BOARD / 'health.h'):08X}U",
           f"-DCAN_PACKET_VERSION_HASH=0x{wire_hash(ROOT / 'opendbc_repo/opendbc/safety/can.h'):08X}U"]
  with tempfile.TemporaryDirectory(prefix="eps48a2-h7-") as directory:
    target = Path(directory)
    linker = (BOARD / "stm32h7/stm32h7x5_flash.ld").read_text()
    old = "FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 1024K"
    if linker.count(old) != 1:
      raise ValueError("unrecognized vendored H7 linker layout")
    # Restrict this build to the application region; do not produce a bootstub.
    (target / "app.ld").write_text(linker.replace(old, "FLASH (rx)         : ORIGIN = 0x08020000, LENGTH = 896K"))
    for source, obj in (("panda/board/main.c", "main.o"), ("panda/board/stm32h7/startup_stm32h7x5xx.s", "startup.o")):
      subprocess.run([str(compiler), *flags, "-c", source, "-o", str(target / obj)], cwd=ROOT, check=True)
    subprocess.run([str(compiler), *flags, "-nostartfiles", "-Wl,--gc-sections", "-T", str(target / "app.ld"),
                    str(target / "startup.o"), str(target / "main.o"), "-lgcc", "-o", str(target / "main.elf")], cwd=ROOT, check=True)
    subprocess.run([str(objcopy), "-O", "binary", str(target / "main.elf"), str(target / "main.bin")], check=True)
    subprocess.run([sys.executable, str(BOARD / "crypto/sign.py"), str(target / "main.bin"),
                    str(target / "app.bin.signed"), str(BOARD / "crypto/certs/debug")], env={**os.environ, "SETLEN": "1"}, check=True)
    data = (target / "app.bin.signed").read_bytes()
    verify_signature(data)
    if len(data) > 896 * 1024:
      raise ValueError("H7 application exceeds its flash region")
    SIGNED.write_bytes(data)
    for name in ("main.elf", "main.bin"):
      shutil.copyfile(target / name, BOARD / "obj/panda_h7" / name)
    compiler_version = subprocess.check_output([str(compiler), "--version"], text=True).splitlines()[0]
  MANIFEST.write_text(json.dumps({"compiler_package": TOOLCHAIN_VERSION, "compiler": compiler_version,
                                  "version": VERSION, "source_sha256": source_digest(), "signed_sha256": digest(data),
                                  "signed_bytes": len(data), "application_address": "0x08020000",
                                  "signing": "existing public development key; existing debug bootstub convention",
                                  "bootstub_modified": False}, indent=2) + "\n")


def check():
  manifest = json.loads(MANIFEST.read_text())
  data = SIGNED.read_bytes()
  verify_signature(data)
  if manifest["source_sha256"] != source_digest() or manifest["signed_sha256"] != digest(data):
    raise ValueError("bundled Panda application does not match the reviewed source/build manifest")
  print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--build", action="store_true")
  parser.add_argument("--check", action="store_true")
  args = parser.parse_args()
  if not (args.build or args.check):
    parser.error("choose --build and/or --check")
  if args.build:
    build()
  if args.check:
    check()
