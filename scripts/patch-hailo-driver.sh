#!/usr/bin/env bash
# Backport Hailo's find_vma() locking fix into the hailo_pci 4.23.0 DKMS source,
# then rebuild, install and reload the module (see the patch file for why).
#
#   scripts/patch-hailo-driver.sh
#
# Safe to re-run: an already patched source is left alone, and DKMS rebuilds the
# patched source after kernel updates. Reinstalling the hailort-pcie-driver
# package restores the unpatched source; run this again afterwards. A 4.24.0 or
# newer driver already contains the fix. Stop anything using the Hailo-8 first.
set -euo pipefail

version=4.23.0
source_dir=/usr/src/hailo_pci-$version
patch_file="$(cd "$(dirname "$0")" && pwd)/hailo_pci-$version-find-vma-lock.patch"

if [[ ! -d $source_dir ]]; then
  echo "No $source_dir: the installed driver is not $version, so this patch does not apply." >&2
  exit 1
fi
if sudo patch --dry-run --reverse --silent -p1 -d "$source_dir" < "$patch_file" >/dev/null 2>&1; then
  echo "$source_dir already has the fix"
else
  sudo patch --forward -p1 -d "$source_dir" < "$patch_file"
fi

sudo dkms build "hailo_pci/$version" --force
sudo dkms install "hailo_pci/$version" --force

# Load the rebuilt module now instead of at the next boot.
sudo systemctl stop hailort.service
sudo modprobe -r hailo_pci
sudo modprobe hailo_pci
sudo systemctl start hailort.service
echo "hailo_pci $version loaded with the fix: srcversion $(cat /sys/module/hailo_pci/srcversion)"
