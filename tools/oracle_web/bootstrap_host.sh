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
policy_path=/usr/sbin/policy-rc.d
policy_backup="$work_dir/policy-rc.d.original"
policy_saved=0
policy_active=0
managed_units=(nginx.service certbot.timer)
unit_was_active=()
unit_was_enabled=()

capture_unit_states() {
	local unit
	for unit in "${managed_units[@]}"; do
		if /usr/bin/systemctl is-active --quiet "$unit"; then
			unit_was_active+=(1)
		else
			unit_was_active+=(0)
		fi
		if /usr/bin/systemctl is-enabled --quiet "$unit"; then
			unit_was_enabled+=(1)
		else
			unit_was_enabled+=(0)
		fi
	done
}

restore_unit_states() {
	local index unit active_now enabled_now prior_active prior_enabled
	for index in "${!managed_units[@]}"; do
		unit=${managed_units[$index]}
		prior_active=${unit_was_active[$index]}
		prior_enabled=${unit_was_enabled[$index]}
		active_now=0
		enabled_now=0
		if /usr/bin/systemctl is-active --quiet "$unit"; then
			active_now=1
		fi
		if /usr/bin/systemctl is-enabled --quiet "$unit"; then
			enabled_now=1
		fi
		if (( active_now != prior_active )); then
			if (( prior_active )); then
				/usr/bin/systemctl start "$unit"
			else
				/usr/bin/systemctl stop "$unit"
			fi
		fi
		if (( enabled_now != prior_enabled )); then
			if (( prior_enabled )); then
				/usr/bin/systemctl enable "$unit"
			else
				/usr/bin/systemctl disable "$unit"
			fi
		fi
	done
}

restore_policy_rc() {
	local status=0
	local operation_status=0
	if (( policy_active )); then
		if rm -f -- "$policy_path"; then
			policy_active=0
		else
			status=$?
		fi
	fi
	if (( policy_saved && ! policy_active )); then
		if cp -a --no-dereference -- "$policy_backup" "$policy_path"; then
			policy_saved=0
		else
			operation_status=$?
			if (( status == 0 )); then
				status=$operation_status
			fi
		fi
	fi
	return "$status"
}

cleanup() {
	local status=$?
	local restore_status=0
	local cleanup_status=0
	trap - EXIT
	set +e
	restore_policy_rc
	restore_status=$?
	if (( restore_status != 0 )); then
		echo "policy-rc.d restoration failed (status $restore_status); recovery material retained at $work_dir" >&2
	else
		find "$work_dir" -mindepth 1 -delete
		cleanup_status=$?
		if (( cleanup_status == 0 )); then
			rmdir "$work_dir"
			cleanup_status=$?
		fi
	fi
	if (( status == 0 && restore_status != 0 )); then
		status=$restore_status
	elif (( status == 0 && cleanup_status != 0 )); then
		status=$cleanup_status
	fi
	exit "$status"
}
trap cleanup EXIT

capture_unit_states

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

if [[ -e "$policy_path" || -L "$policy_path" ]]; then
	cp -a --no-dereference -- "$policy_path" "$policy_backup"
	policy_saved=1
fi
policy_source="$work_dir/policy-rc.d"
printf '%s\n' '#!/bin/sh' 'exit 101' >"$policy_source"
policy_active=1
rm -f -- "$policy_path"
install -o root -g root -m 0755 -- "$policy_source" "$policy_path"

/usr/bin/apt-get update
DEBIAN_FRONTEND=noninteractive /usr/bin/apt-get install --yes \
	nginx certbot python3-certbot-nginx curl rsync
restore_policy_rc
restore_unit_states

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
