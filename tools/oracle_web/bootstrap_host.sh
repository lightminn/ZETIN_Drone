#!/usr/bin/env bash
set -euo pipefail

if (( EUID != 0 )); then
	echo "bootstrap_host.sh must run as root" >&2
	exit 1
fi

source_root=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
install_root=/usr/local/lib/zetin-web/oracle_web
backup_root=/var/backups/zetin-web
backup_stamp=$(date -u +%Y%m%dT%H%M%SZ)-$$
work_dir=$(mktemp -d /var/tmp/zetin-web-bootstrap.XXXXXX)
trap 'find "$work_dir" -mindepth 1 -delete; rmdir "$work_dir"' EXIT

backup_target() {
	local target=$1
	local backup="$backup_root/$backup_stamp/${target#/}"
	install -d -o root -g root -m 0700 "$(dirname -- "$backup")"
	install -o root -g root -m 0600 -- "$target" "$backup"
}

install_file() {
	local source=$1
	local target=$2
	local mode=$3
	install -d -o root -g root -m 0755 "$(dirname -- "$target")"
	if [[ -e "$target" || -L "$target" ]]; then
		if [[ ! -f "$target" || -L "$target" ]]; then
			echo "refusing to replace non-regular target: $target" >&2
			return 1
		fi
		if ! cmp -s -- "$source" "$target"; then
			backup_target "$target"
			install -o root -g root -m "$mode" -- "$source" "$target"
		else
			chown root:root -- "$target"
			chmod "$mode" -- "$target"
		fi
	else
		install -o root -g root -m "$mode" -- "$source" "$target"
	fi
}

/usr/bin/apt-get update
DEBIAN_FRONTEND=noninteractive /usr/bin/apt-get install --yes \
	nginx certbot python3-certbot-nginx curl rsync

install -d -o root -g root -m 0755 \
	/srv/zetin-web/apps \
	/etc/zetin-web \
	/etc/zetin-web/tls \
	/var/lib/zetin-web

while IFS= read -r -d '' source; do
	relative=${source#"$source_root"/}
	case "$relative" in
		*.pyc|*/__pycache__/*)
			continue
			;;
		*.sh|*.run)
			mode=0755
			;;
		*)
			mode=0644
			;;
	esac
	install_file "$source" "$install_root/$relative" "$mode"
done < <(find "$source_root" -type f -print0)

release_wrapper="$work_dir/zetin-web-release"
firewall_wrapper="$work_dir/zetin-web-firewall"
printf '%s\n' \
	'#!/bin/sh' \
	'PYTHONPATH=/usr/local/lib/zetin-web exec /usr/bin/python3 -m oracle_web.host_release "$@"' \
	>"$release_wrapper"
printf '%s\n' \
	'#!/bin/sh' \
	'PYTHONPATH=/usr/local/lib/zetin-web exec /usr/bin/python3 -m oracle_web.host_firewall "$@"' \
	>"$firewall_wrapper"
install_file "$release_wrapper" /usr/local/sbin/zetin-web-release 0755
install_file "$firewall_wrapper" /usr/local/sbin/zetin-web-firewall 0755

install_file \
	"$source_root/templates/zetin-webapp@.service" \
	/etc/systemd/system/zetin-webapp@.service \
	0644
install_file \
	"$source_root/templates/nginx-limits.conf" \
	/etc/nginx/conf.d/zetin-web-limits.conf \
	0644

/usr/bin/systemctl daemon-reload
/usr/sbin/nginx -t
