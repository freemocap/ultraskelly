"""Build a candidate offline client or vendor the client pinned with the Python stream package."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import tomllib
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / ".cache" / "sdk-candidate"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdk", type=Path, help="Local freemocap/sdk Git repository")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--revision", help="Build an alternate SDK revision for replay")
    mode.add_argument("--vendor", action="store_true", help="Install the bundle for the pyproject.toml pin")
    args = parser.parse_args()
    with (ROOT / "pyproject.toml").open("rb") as project:
        pin = tomllib.load(project)["tool"]["uv"]["sources"]["freemocap-sdk-stream"]["rev"]
    revision = subprocess.check_output(
        ["git", "-C", args.sdk, "rev-parse", "--verify", "--end-of-options", (args.revision or pin) + "^{commit}"], text=True
    ).strip()
    out = OUTPUT / revision
    out.mkdir(parents=True, exist_ok=True)
    npm = shutil.which("npm.cmd") or shutil.which("npm")
    if not npm:
        parser.error("npm and Node 22 or later are required")
    with tempfile.TemporaryDirectory(dir=out) as temporary:
        work = Path(temporary)
        archive = work / "source.zip"
        subprocess.run(["git", "-C", args.sdk, "archive", "--format=zip", "-o", archive, revision,
                        "packages/client"], check=True)
        with ZipFile(archive) as source:
            source.extractall(work)
        client = work / "packages/client"
        subprocess.run([npm, "ci", "--ignore-scripts", "--no-audit", "--no-fund"], cwd=client, check=True)
        subprocess.run([npm, "run", "build"], cwd=client, check=True)
        # Resolve the package's public entry point. Future SDK channel exports then reach the same test.
        entry = client / "candidate.mjs"
        entry.write_text("export * from '@freemocap/client';\n", encoding="utf-8")
        bundle = client / "freemocap-client.js"
        flags = ["--bundle", "--format=esm", "--platform=browser", "--minify", "--legal-comments=inline", "--sourcemap=linked"]
        subprocess.run(["node", "node_modules/esbuild/bin/esbuild", entry.name, *flags,
                        "--outfile=" + bundle.name], cwd=client, check=True)
        manifest = {"sdk_revision": revision, "package": "@freemocap/client",
                    "package_lock_sha256": sha256(client / "package-lock.json"), "bundle_sha256": sha256(bundle),
                    "node": subprocess.check_output(["node", "--version"], text=True).strip(),
                    "npm": subprocess.check_output([npm, "--version"], text=True).strip(), "esbuild_flags": flags}
        bundle.replace(out / bundle.name)
        (client / "freemocap-client.js.map").replace(out / "freemocap-client.js.map")
        (out / "build.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if args.vendor:
        vendor = ROOT / "ultraskelly/ui/dashboard/vendor"
        for name in ("freemocap-client.js", "freemocap-client.js.map"):
            shutil.copyfile(out / name, vendor / name)
        manifest.update(repository="https://github.com/freemocap/sdk", license="AGPL-3.0-or-later",
                        bundled_licenses={"cbor-x": "MIT", "zod": "MIT"},
                        rebuild="scripts/update-freemocap-client.sh /path/to/sdk")
        (vendor / "freemocap-client.SOURCE").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(out)


if __name__ == "__main__":
    main()
