#!/usr/bin/env bash
# Set up a Raspberry Pi 5 to run the robot, or check that it is set up.
#
#   scripts/setup-pi.sh --check             report every setting; change nothing
#   scripts/setup-pi.sh                     put in place whatever is missing, then report
#   scripts/setup-pi.sh --enable-services   also start the robot and its dashboard at every boot
#
# Run it on the Pi from the checkout the robot should run (normally ~/ultraskelly),
# as the user the robot runs as. It expects Raspberry Pi OS Trixie with the desktop,
# imaged with Raspberry Pi Imager (user, Wi-Fi and SSH set there). Whatever is in
# place is left alone, so it is safe to re-run. The first sudo may ask for the
# password. Files under scripts/pi-system/ are installed at the same paths under /.
# The robot services keep their start-at-boot setting unless you pass
# --enable-services; the bot moves the motors when it starts.
#
# Each line is one setting or observation: OK; FIXED (put in place just now);
# MISSING (run without --check to fix it); FAILED; WARN (hardware, power or network
# for a person to look at); SKIPPED. The exit status is 1 if anything is MISSING or
# FAILED.
set -uo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
system_files=$repo/scripts/pi-system
config=/boot/firmware/config.txt
lock=/run/lock/ultraskelly-robot.lock
user="$(id -un)"
uv="$(command -v uv || echo "$HOME/.local/bin/uv")"
packages=(git curl patch chromium i2c-tools python3-munkres hailo-all)

mode=apply
enable_services=
for arg in "$@"; do
  case $arg in
    --check) mode=check ;;
    --enable-services) enable_services=--enable ;;
    *) echo "usage: $0 [--check | --enable-services]" >&2; exit 2 ;;
  esac
done
if [[ $mode == check && -n $enable_services ]]; then
  echo "--check changes nothing, so it cannot enable the services." >&2
  exit 2
fi
if [[ $user == root ]]; then
  echo "Run this as the robot's user, not as root; it uses sudo where it must." >&2
  exit 1
fi

declare -i ok=0 fixed=0 missing=0 failed=0 warnings=0 skipped=0
reboot_for=()

say() { printf '%-8s %-21s %s\n' "$1" "$2" "$3"; }
list() {
  local text
  text=$(printf '%s, ' "$@")
  echo "${text%, }"
}

# setting NAME CHECK [FIX]: CHECK prints a short detail and succeeds when the setting
# is in place. Unless this is --check, a failing CHECK runs FIX, then CHECK again.
setting() {
  local name=$1 check=$2 fix=${3:-} detail
  if detail=$("$check" 2>&1); then say OK "$name" "$detail"; ok+=1; return; fi
  if [[ $mode == apply && -n $fix ]]; then
    echo "== $name: $detail. Fixing..." >&2
    if ! "$fix"; then say FAILED "$name" "$detail"; failed+=1; return; fi
    if detail=$("$check" 2>&1); then say FIXED "$name" "$detail"; fixed+=1; return; fi
  fi
  say MISSING "$name" "$detail"
  missing+=1
}

# observe NAME CHECK: hardware, power or network for a person to see to. CHECK
# succeeds when all is well, returns 1 for a warning and 2 when it could not look.
observe() {
  local name=$1 check=$2 detail status
  detail=$("$check" 2>&1)
  status=$?
  case $status in
    0) say OK "$name" "$detail"; ok+=1 ;;
    2) say SKIPPED "$name" "$detail"; skipped+=1 ;;
    *) say WARN "$name" "$detail"; warnings+=1 ;;
  esac
}

# A file from scripts/pi-system/ is in place when it is identical at the same path under /.
file_in_place() {
  cmp -s "$system_files$1" "$1" || { echo "$1 is missing or differs from scripts/pi-system"; return 1; }
}
install_file() {
  local permissions=644
  [[ -x $system_files$1 ]] && permissions=755
  sudo install -D -m "$permissions" "$system_files$1" "$1"
}
hailo_pci_device() {
  local device
  for device in /sys/bus/pci/devices/*; do
    [[ $(<"$device/vendor") == 0x1e60 ]] && { echo "$device"; return; }
  done
  return 1
}

# --- Settings -------------------------------------------------------------------

check_platform() {
  local model os
  model=$(tr -d '\0' </proc/device-tree/model 2>/dev/null)
  # shellcheck source=/dev/null
  os=$(. /etc/os-release && echo "$PRETTY_NAME")
  echo "$model; $os; kernel $(uname -r)"
  [[ $model == "Raspberry Pi 5"* && $os == *trixie* ]]
}

check_sudo() {
  sudo -n true 2>/dev/null || { echo "sudo asks for a password"; return 1; }
  echo "sudo works without a password, for maintenance over SSH"
}
fix_sudo() {
  local rule=/etc/sudoers.d/zz-ultraskelly-nopasswd
  # sudo ignores names with a dot, so the draft does nothing until checked and renamed.
  echo "$user ALL=(ALL) NOPASSWD: ALL" | sudo tee "$rule.new" >/dev/null &&
    sudo chmod 440 "$rule.new" && sudo visudo -cqf "$rule.new" && sudo mv "$rule.new" "$rule"
}

check_dpkg() {
  [[ -z $(dpkg --audit 2>&1) ]] || { echo "half-installed packages, as after a power loss during an install"; return 1; }
  echo "no half-installed packages"
}
fix_dpkg() { sudo dpkg --configure -a; }

missing_packages() {
  local package
  for package in "${packages[@]}"; do
    [[ $(dpkg-query -W -f='${Status}' "$package" 2>/dev/null) == "install ok installed" ]] || echo "$package"
  done
}
check_packages() {
  local absent
  absent=$(missing_packages | tr '\n' ' ')
  [[ -z $absent ]] || { echo "not installed: $absent"; return 1; }
  echo "${packages[*]}; HailoRT $(dpkg-query -W -f='${Version}' hailort)"
}
fix_packages() {
  local absent
  mapfile -t absent < <(missing_packages)
  sudo apt-get update && sudo apt-get install -y "${absent[@]}" || return 1
  [[ " ${absent[*]} " == *" hailo-all "* ]] && reboot_for+=("Hailo driver")
  return 0
}

hailo_packages() {
  dpkg-query -W -f='${Package} ${Status}\n' 'hailo*' 'python3-hailo*' 2>/dev/null | awk '$NF == "installed" {print $1}'
}
check_hailo_hold() {
  local installed unheld
  installed=$(hailo_packages)
  [[ -n $installed ]] || { echo "no Hailo packages installed"; return 1; }
  unheld=$(comm -23 <(sort <<<"$installed") <(apt-mark showhold | sort) | tr '\n' ' ')
  [[ -z $unheld ]] || { echo "not held, so an apt upgrade could replace them: $unheld"; return 1; }
  echo "$(wc -l <<<"$installed") Hailo packages held at their installed versions"
}
fix_hailo_hold() {
  local installed
  mapfile -t installed < <(hailo_packages)
  sudo apt-mark hold "${installed[@]}"
}

check_hailo_driver() {
  local version loaded installed
  version=$(dpkg-query -W -f='${Version}' hailort-pcie-driver 2>/dev/null)
  [[ -n $version ]] || { echo "hailort-pcie-driver is not installed"; return 1; }
  if [[ $version == 4.23.0* ]]; then
    patch --dry-run --reverse --silent -p1 -d /usr/src/hailo_pci-4.23.0 \
      <"$repo/scripts/hailo_pci-4.23.0-find-vma-lock.patch" >/dev/null 2>&1 ||
      { echo "hailo_pci 4.23.0 lacks the find_vma lock fix"; return 1; }
  elif dpkg --compare-versions "$version" lt 4.24.0; then
    echo "hailo_pci $version lacks the find_vma lock fix, and the patch is for 4.23.0"
    return 1
  fi
  loaded=$(cat /sys/module/hailo_pci/srcversion 2>/dev/null)
  installed=$(/usr/sbin/modinfo -F srcversion hailo_pci 2>/dev/null)
  [[ -n $loaded && $loaded == "$installed" ]] ||
    { echo "the loaded hailo_pci (${loaded:-none}) is not the installed build (${installed:-none})"; return 1; }
  echo "hailo_pci $version with the find_vma lock fix, loaded (srcversion $loaded)"
}
fix_hailo_driver() {
  flock -n -E 75 "$lock" "$repo/scripts/patch-hailo-driver.sh"
  local status=$?
  ((status == 75)) && echo "The robot is in use ($lock is held); stop it and run this again." >&2
  return "$status"
}

check_i2c() {
  [[ $(raspi-config nonint get_i2c) == 0 ]] || { echo "I2C is off in config.txt"; return 1; }
  [[ -e /dev/i2c-1 ]] || { echo "on in config.txt; /dev/i2c-1 appears after a reboot"; return 0; }
  echo "on: /dev/i2c-1"
}
fix_i2c() { sudo raspi-config nonint do_i2c 0 && reboot_for+=(I2C); }

check_pcie() {
  grep -qx 'dtparam=pciex1_gen=3' "$config" || { echo "$config lacks dtparam=pciex1_gen=3"; return 1; }
  echo "dtparam=pciex1_gen=3 in config.txt, for the Hailo-8"
}
fix_pcie() {
  printf '\n[all]\n# Run the Hailo-8 AI HAT+ at PCIe Gen 3\ndtparam=pciex1_gen=3\n' | sudo tee -a "$config" >/dev/null &&
    reboot_for+=("PCIe Gen 3")
}

check_oak_rule() {
  file_in_place /etc/udev/rules.d/80-movidius.rules || return 1
  echo "the bot may open the OAK without root"
}
fix_oak_rule() {
  install_file /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload &&
    sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=03e7
}

check_journal() {
  file_in_place /etc/systemd/journald.conf.d/90-ultraskelly-persistent.conf || return 1
  [[ -d /var/log/journal ]] || { echo "no /var/log/journal"; return 1; }
  echo "kept on the card ($(journalctl --disk-usage | grep -o '[0-9.]*[KMG]')), synced every 10 s"
}
fix_journal() {
  install_file /etc/systemd/journald.conf.d/90-ultraskelly-persistent.conf && sudo systemctl restart systemd-journald
}

check_boot_report() {
  file_in_place /usr/local/sbin/ultraskelly-boot-report || return 1
  file_in_place /etc/systemd/system/ultraskelly-boot-report.service || return 1
  systemctl -q is-enabled ultraskelly-boot-report.service || { echo "ultraskelly-boot-report is not enabled"; return 1; }
  echo "each boot logs why the Pi last stopped (journalctl -u ultraskelly-boot-report)"
}
fix_boot_report() {
  install_file /usr/local/sbin/ultraskelly-boot-report &&
    install_file /etc/systemd/system/ultraskelly-boot-report.service &&
    sudo systemctl daemon-reload && sudo systemctl enable --now ultraskelly-boot-report.service
}

check_watchdog() {
  file_in_place /etc/systemd/system.conf.d/90-ultraskelly-watchdog.conf || return 1
  local timeout
  timeout=$(systemctl show -p RuntimeWatchdogUSec --value)
  [[ $timeout != 0 && $timeout != infinity && -e /dev/watchdog0 ]] ||
    { echo "systemd does not run the hardware watchdog (RuntimeWatchdogUSec=$timeout)"; return 1; }
  echo "a hang resets the Pi after $timeout"
}
fix_watchdog() {
  install_file /etc/systemd/system.conf.d/90-ultraskelly-watchdog.conf && sudo systemctl daemon-reexec
}

check_wifi_power_save() {
  file_in_place /etc/NetworkManager/conf.d/90-ultraskelly-wifi.conf || return 1
  [[ $(/usr/sbin/iw dev wlan0 get power_save 2>/dev/null) != *on ]] || { echo "still on for the current connection"; return 1; }
  echo "off"
}
fix_wifi_power_save() {
  install_file /etc/NetworkManager/conf.d/90-ultraskelly-wifi.conf && sudo systemctl reload NetworkManager || return 1
  [[ ! -e /sys/class/net/wlan0 ]] || sudo /usr/sbin/iw dev wlan0 set power_save off
}

empty_profiles() {
  find /etc/netplan /etc/NetworkManager/system-connections -type f -size 0 2>/dev/null
}
check_network_profiles() {
  local empty
  empty=$(empty_profiles | tr '\n' ' ')
  [[ -z $empty ]] || { echo "empty, as after a power loss mid-write: $empty"; return 1; }
  echo "no empty network profiles"
}
fix_network_profiles() {
  local empty
  mapfile -t empty < <(empty_profiles)
  sudo rm -- "${empty[@]}" && sudo nmcli connection reload
}

check_desktop() {
  [[ $(raspi-config nonint get_boot_cli) == 1 ]] || { echo "boots to the console, not the desktop"; return 1; }
  [[ $(raspi-config nonint get_autologin) == 0 ]] || { echo "the desktop waits for a login"; return 1; }
  echo "boots to the desktop and logs $user in, so the dashboard can open"
}
fix_desktop() { sudo raspi-config nonint do_boot_behaviour B4 && reboot_for+=("desktop login"); }

check_blanking() {
  [[ $(raspi-config nonint get_blanking) == 1 ]] || { echo "the screen blanks when idle"; return 1; }
  echo "the screen stays on"
}
fix_blanking() { sudo raspi-config nonint do_blanking 1 && reboot_for+=("screen blanking"); }

check_linger() {
  [[ -e /var/lib/systemd/linger/$user ]] || { echo "$user's services wait for a login"; return 1; }
  echo "$user's services start at boot"
}
fix_linger() { sudo loginctl enable-linger "$user"; }

check_uv() {
  [[ -x $uv ]] || { echo "uv is not installed"; return 1; }
  "$uv" --version
}
fix_uv() { curl -LsSf https://astral.sh/uv/install.sh | sh; }

venv_on_system_python() {
  [[ $(readlink -f "$repo/.venv/bin/python") == /usr/bin/python3* ]] &&
    grep -qx 'include-system-site-packages = true' "$repo/.venv/pyvenv.cfg" 2>/dev/null
}
check_venv() {
  local python=$repo/.venv/bin/python
  [[ -x $python ]] || { echo "no virtual environment at $repo/.venv"; return 1; }
  venv_on_system_python || { echo ".venv is not built on the system Python with its packages (Hailo)"; return 1; }
  (cd "$repo" && "$uv" sync --locked --extra hardware --python /usr/bin/python3 --check) >/dev/null 2>&1 ||
    { echo ".venv does not match uv.lock"; return 1; }
  "$python" -c 'import ultraskelly.bot_launcher, depthai, hailo_platform, cv2, adafruit_motorkit, adafruit_servokit' \
    >/dev/null 2>&1 || { echo "the bot's imports fail in .venv"; return 1; }
  echo "$("$python" -V) with the system packages; matches uv.lock; the bot's imports load"
}
fix_venv() (
  cd "$repo" || exit 1
  if [[ -e .venv ]] && ! venv_on_system_python; then
    rm -rf .venv  # uv.lock rebuilds it exactly
  fi
  if [[ ! -x .venv/bin/python ]]; then
    "$uv" venv --system-site-packages --python /usr/bin/python3 || exit 1
  fi
  "$uv" sync --locked --extra hardware --python /usr/bin/python3
)

check_services() {
  "$repo/scripts/install-services.sh" --check >/dev/null 2>&1 || { echo "not installed for $repo"; return 1; }
  local bot dashboard
  bot="$(systemctl is-enabled ultraskelly-bot.service), $(systemctl is-active ultraskelly-bot.service)"
  dashboard="$(systemctl --user is-enabled ultraskelly-dashboard.service), $(systemctl --user is-active ultraskelly-dashboard.service)"
  if [[ -n $enable_services && ($bot != enabled* || $dashboard != enabled*) ]]; then
    echo "installed but not enabled: bot $bot; dashboard $dashboard"
    return 1
  fi
  echo "installed for this checkout: bot $bot; dashboard $dashboard"
}
fix_services() { "$repo/scripts/install-services.sh" ${enable_services:+"$enable_services"}; }

# --- Observations ---------------------------------------------------------------

check_last_stop() {
  local report
  report=$("$system_files/usr/local/sbin/ultraskelly-boot-report" 2>/dev/null)
  echo "${report#<[0-9]>Last stop: }"
  [[ $report != "<4>"* ]]
}

check_power() {
  local flags bit notes=()
  local -A meaning=(
    [0x1]="undervoltage now" [0x2]="frequency capped now" [0x4]="throttled now"
    [0x8]="temperature limit now" [0x10000]="undervoltage since boot"
    [0x20000]="frequency capped since boot" [0x40000]="throttled since boot"
    [0x80000]="temperature limit since boot"
  )
  flags=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
  [[ -n $flags ]] || { echo "vcgencmd get_throttled did not answer"; return 2; }
  ((flags == 0)) && { echo "no undervoltage, throttling or temperature limit since boot"; return 0; }
  for bit in 0x1 0x2 0x4 0x8 0x10000 0x20000 0x40000 0x80000; do
    ((flags & bit)) && notes+=("${meaning[$bit]}")
  done
  echo "throttled=$flags: $(list "${notes[@]}")"
  return 1
}

check_usb_power() {
  local enabled supply
  enabled=$(vcgencmd get_config usb_max_current_enable 2>/dev/null | cut -d= -f2)
  supply=$(od -An -tu4 --endian=big /proc/device-tree/chosen/power/max_current 2>/dev/null | tr -d ' ')
  [[ $enabled == 1 ]] ||
    { echo "the USB ports share 0.6 A; the supply reports ${supply:-no} mA, and 1.6 A needs a 5 A USB-PD supply"; return 1; }
  echo "the USB ports may draw 1.6 A together; the supply reports $supply mA"
}

check_usb_devices() {
  local device id found=() absent=() zed_hid='' total=0 power
  for device in /sys/bus/usb/devices/*-*; do
    [[ $device != *:* && -f $device/idVendor ]] || continue
    id=$(<"$device/idVendor"):$(<"$device/idProduct")
    power=$(<"$device/bMaxPower")
    total=$((total + ${power%mA}))
    case $id in
      03e7:*) found+=(OAK) ;;
      2b03:f682) found+=("ZED Mini") ;;
      2b03:f681) zed_hid=yes ;;
      046d:0aaf) found+=("Yeti X") ;;
    esac
  done
  [[ " ${found[*]} " == *" OAK "* ]] || absent+=(OAK)
  if [[ " ${found[*]} " != *" ZED Mini "* ]]; then
    absent+=("ZED Mini video${zed_hid:+ (only its IMU is up: flip its USB-C plug at the camera)}")
  fi
  local declared="devices declare $total of 1600 mA"
  ((${#absent[@]} == 0)) || { echo "missing: $(list "${absent[@]}"); $declared"; return 1; }
  echo "$(list "${found[@]}"); $declared"
}

check_hailo_device() {
  local device speed
  device=$(hailo_pci_device) || { echo "no Hailo-8 on PCIe: is the AI HAT+ seated?"; return 1; }
  speed=$(<"$device/current_link_speed")
  [[ -e /dev/hailo0 ]] || { echo "on PCIe but no /dev/hailo0: is hailo_pci loaded?"; return 1; }
  [[ $speed == 8.0* ]] || { echo "/dev/hailo0 on a $speed link, not Gen 3 (8.0 GT/s)"; return 1; }
  echo "/dev/hailo0 on a ${speed% PCIe} link"
}

check_i2c_boards() {
  [[ -e /dev/i2c-1 ]] || { echo "no /dev/i2c-1"; return 1; }
  [[ -e $lock ]] || : >"$lock"
  exec 9<"$lock"
  flock -n 9 || { echo "not probed: the robot is using the boards"; return 2; }
  local address absent=()
  for address in 0x40 0x60; do
    /usr/sbin/i2cget -y 1 "$address" 0x00 >/dev/null 2>&1 || absent+=("$address")
  done
  ((${#absent[@]} == 0)) ||
    { echo "no answer at ${absent[*]} (0x40 is the servo board, 0x60 the motor board)"; return 1; }
  echo "the servo board (0x40) and the motor board (0x60) answer"
}

check_wifi() {
  local radio country connection
  radio=$(nmcli -t radio wifi 2>/dev/null)
  [[ $radio == enabled ]] || { echo "the Wi-Fi radio is ${radio:-unknown}; turn it on: nmcli radio wifi on"; return 1; }
  nmcli -t -f TYPE,AUTOCONNECT connection show | grep -q '^802-11-wireless:yes$' ||
    { echo "no saved Wi-Fi network; add one from the desktop's network menu"; return 1; }
  country=$(/usr/sbin/iw reg get | awk '/^country/ {sub(":", "", $2); print $2; exit}')
  [[ -n $country && $country != 00 ]] || { echo "no Wi-Fi country set: sudo raspi-config nonint do_wifi_country US"; return 1; }
  connection=$(nmcli -t -f ACTIVE,SSID,FREQ,SIGNAL device wifi list --rescan no |
    awk -F: '$1 == "yes" {print $2 ", " $3 ", signal " $4 "%"}')
  [[ -n $connection ]] || { echo "not connected to Wi-Fi"; return 1; }
  echo "$connection; country $country; Ethernet $(nmcli -t -f DEVICE,STATE device | awk -F: '$1 == "eth0" {print $2}')"
}

check_repo() {
  local objects empty
  objects=$(git -C "$repo" rev-parse --path-format=absolute --git-common-dir)/objects
  empty=$(find "$objects" -type f -empty 2>/dev/null | head -n 3 | tr '\n' ' ')
  [[ -z $empty ]] || { echo "empty git objects, as after a power loss mid-write: $empty"; return 1; }
  git -C "$repo" fsck --connectivity-only --no-progress >/dev/null 2>&1 || { echo "git fsck finds damage"; return 1; }
  echo "$(git -C "$repo" rev-parse --abbrev-ref HEAD) at $(git -C "$repo" rev-parse --short HEAD); no damaged objects"
}

# --- Run ------------------------------------------------------------------------

echo "UltraSkelly Pi setup ($mode) of $repo on $(hostname), $(date '+%Y-%m-%d %H:%M')"
observe "Raspberry Pi" check_platform
setting "Passwordless sudo" check_sudo fix_sudo
setting "Package database" check_dpkg fix_dpkg
setting "Packages" check_packages fix_packages
setting "Hailo packages held" check_hailo_hold fix_hailo_hold
setting "Hailo driver fix" check_hailo_driver fix_hailo_driver
setting "I2C" check_i2c fix_i2c
setting "PCIe Gen 3" check_pcie fix_pcie
setting "OAK udev rule" check_oak_rule fix_oak_rule
setting "Persistent journal" check_journal fix_journal
setting "Boot report" check_boot_report fix_boot_report
setting "Hardware watchdog" check_watchdog fix_watchdog
setting "Wi-Fi power save" check_wifi_power_save fix_wifi_power_save
setting "Network profiles" check_network_profiles fix_network_profiles
setting "Desktop login" check_desktop fix_desktop
setting "Screen blanking" check_blanking fix_blanking
setting "Services at boot" check_linger fix_linger
setting "uv" check_uv fix_uv
setting "Python environment" check_venv fix_venv
setting "Robot services" check_services fix_services
observe "Last stop" check_last_stop
observe "Power" check_power
observe "USB power" check_usb_power
observe "USB devices" check_usb_devices
observe "Hailo-8" check_hailo_device
observe "I2C boards" check_i2c_boards
observe "Wi-Fi" check_wifi
observe "Repository" check_repo

echo
echo "$ok OK, $fixed fixed, $missing missing, $failed failed, $warnings warnings, $skipped skipped"
if ((${#reboot_for[@]})); then
  echo "Reboot to finish setting up: $(list "${reboot_for[@]}")"
fi
((missing + failed == 0))
