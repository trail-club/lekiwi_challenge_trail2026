#!/usr/bin/env bash
# 機体固有のUSBシリアルを .env から受け取り、ホストのudevルールを生成する。
# 追跡対象のテンプレート自体は書き換えない。
set -euo pipefail

usage() {
  echo "usage: $0 [--dry-run] split|shared" >&2
  exit 2
}

dry_run=false
if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=true
  shift
fi
mode="${1:-}"
[[ "$mode" == "split" || "$mode" == "shared" ]] || usage
[[ $# -eq 1 ]] || usage

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/../.." && pwd)"
lekiwi_template="$repo_dir/docker/lekiwi_base_ros2/99-lekiwi.rules"
so101_template="$repo_dir/docker/so101_ros2/99-so101.rules"
rplidar_rule="$repo_dir/docker/rplidar_ros2/99-rplidar.rules"

validate_serial() {
  local name="$1"
  local value="$2"
  if [[ -z "$value" ]]; then
    echo "ERROR: $name が空です。docker/robot/.env に実機の ID_SERIAL_SHORT を設定してください。" >&2
    exit 2
  fi
  if [[ ! "$value" =~ ^[A-Za-z0-9._:-]+$ ]]; then
    echo "ERROR: $name にudevルールへ安全に埋め込めない文字が含まれています: $value" >&2
    exit 2
  fi
}

validate_serial LEKIWI_SERIAL "${LEKIWI_SERIAL:-}"
if [[ "$mode" == "split" ]]; then
  validate_serial SO101_SERIAL "${SO101_SERIAL:-}"
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT
sed "s/@LEKIWI_SERIAL@/${LEKIWI_SERIAL}/g" "$lekiwi_template" > "$tmp_dir/99-lekiwi.rules"
cp "$rplidar_rule" "$tmp_dir/99-rplidar.rules"
if [[ "$mode" == "split" ]]; then
  sed "s/@SO101_SERIAL@/${SO101_SERIAL}/g" "$so101_template" > "$tmp_dir/99-so101.rules"
fi

if [[ "$dry_run" == true ]]; then
  for rule in "$tmp_dir"/*.rules; do
    echo "--- $(basename "$rule") ---"
    sed -n '/^SUBSYSTEM/p' "$rule"
  done
  exit 0
fi

for device_name in lekiwi rplidar; do
  device_path="/dev/$device_name"
  if [[ -d "$device_path" && ! -L "$device_path" ]]; then
    echo "ERROR: $device_path がディレクトリです。先に次を実行してください:" >&2
    echo "  sudo rmdir $device_path" >&2
    exit 1
  fi
done
if [[ "$mode" == "split" && -d /dev/so101_follower && ! -L /dev/so101_follower ]]; then
  echo "ERROR: /dev/so101_follower がディレクトリです。先に次を実行してください:" >&2
  echo "  sudo rmdir /dev/so101_follower" >&2
  exit 1
fi

sudo install -m 0644 "$tmp_dir/99-lekiwi.rules" /etc/udev/rules.d/99-lekiwi.rules
sudo install -m 0644 "$tmp_dir/99-rplidar.rules" /etc/udev/rules.d/99-rplidar.rules
if [[ "$mode" == "split" ]]; then
  sudo install -m 0644 "$tmp_dir/99-so101.rules" /etc/udev/rules.d/99-so101.rules
else
  # shared機で過去のsplit用ルールを勝手に消さない。別機体との併用を壊さないため。
  true
fi

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
echo "udevルールをインストールしました ($mode)。"
if [[ "$mode" == "split" ]]; then
  echo "確認: ls -l /dev/lekiwi /dev/so101_follower /dev/rplidar"
else
  echo "確認: ls -l /dev/lekiwi /dev/rplidar"
fi
