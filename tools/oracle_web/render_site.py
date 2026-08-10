"""Render strict per-site Nginx and environment configuration."""

from __future__ import annotations

import argparse
import os
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Mapping, Sequence

from .site_manifest import ManifestError, load_site_manifest


TEMPLATE_ROOT = Path(__file__).resolve().parent / "templates"
ALLOWED_TOKENS = frozenset({
	"@@SITE@@",
	"@@DOMAIN@@",
	"@@PORT@@",
	"@@CERTIFICATE@@",
	"@@PRIVATE_KEY@@",
})
TOKEN_RE = re.compile(r"@@[A-Z_]+@@")
SAFE_ABSOLUTE_PATH_RE = re.compile(r"/[A-Za-z0-9._/-]+\Z")


class RenderError(RuntimeError):
	"""Raised when site configuration cannot be rendered safely."""


def _validate_absolute_path(value: str, label: str) -> str:
	if not isinstance(value, str) or not SAFE_ABSOLUTE_PATH_RE.fullmatch(value):
		raise RenderError(f"{label} must be a safe absolute path")
	parsed = PurePosixPath(value)
	if not parsed.is_absolute() or any(part in (".", "..") for part in parsed.parts):
		raise RenderError(f"{label} must be a canonical absolute path")
	return value


def render_template(template: str, replacements: Mapping[str, str]) -> str:
	"""Replace only the declared token set, rejecting incomplete templates."""
	unknown_replacements = set(replacements) - ALLOWED_TOKENS
	if unknown_replacements:
		raise RenderError(f"unknown replacement tokens: {sorted(unknown_replacements)}")
	found = set(TOKEN_RE.findall(template))
	unknown_tokens = found - ALLOWED_TOKENS
	missing_tokens = set(replacements) - found
	if unknown_tokens or missing_tokens or "@@" in TOKEN_RE.sub("", template):
		parts = []
		if unknown_tokens:
			parts.append(f"unknown tokens: {sorted(unknown_tokens)}")
		if missing_tokens:
			parts.append(f"missing tokens: {sorted(missing_tokens)}")
		if "@@" in TOKEN_RE.sub("", template):
			parts.append("malformed token marker")
		raise RenderError("; ".join(parts))
	rendered = template
	for token, value in replacements.items():
		rendered = rendered.replace(token, value)
	if TOKEN_RE.search(rendered) or "@@" in rendered:
		raise RenderError("rendered template contains an unresolved token")
	return rendered


def render_site(
	site_config: Path,
	certificate: str,
	private_key: str,
	output_dir: Path,
	*,
	template_root: Path = TEMPLATE_ROOT,
) -> tuple[Path, Path]:
	"""Validate one manifest and write its Nginx site and environment files."""
	manifest = load_site_manifest(site_config)
	if manifest.backend is None:
		raise RenderError("site manifest must define a backend")
	if not 1024 <= manifest.backend.port <= 65535:
		raise RenderError("backend port must be an integer from 1024 through 65535")
	certificate = _validate_absolute_path(certificate, "certificate")
	private_key = _validate_absolute_path(private_key, "private key")
	replacements = {
		"@@SITE@@": manifest.site,
		"@@DOMAIN@@": manifest.server_name,
		"@@PORT@@": str(manifest.backend.port),
		"@@CERTIFICATE@@": certificate,
		"@@PRIVATE_KEY@@": private_key,
	}
	try:
		template = (template_root / "nginx-site.conf").read_text(encoding="utf-8")
	except OSError as error:
		raise RenderError(f"cannot read Nginx template: {error}") from error
	rendered = render_template(template, replacements)
	output_dir.mkdir(parents=True, exist_ok=True)
	site_path = output_dir / f"{manifest.site}.conf"
	env_path = output_dir / f"{manifest.site}.env"
	site_path.write_text(rendered, encoding="utf-8")
	env_path.write_text(f"ZETIN_WEB_PORT={manifest.backend.port}\n", encoding="utf-8")
	os.chmod(site_path, 0o644)
	os.chmod(env_path, 0o644)
	return site_path, env_path


def _parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--site-config", required=True, type=Path)
	parser.add_argument("--certificate", required=True)
	parser.add_argument("--private-key", required=True)
	parser.add_argument("--output-dir", required=True, type=Path)
	return parser


def main(argv: Sequence[str] | None = None) -> int:
	args = _parser().parse_args(argv)
	try:
		render_site(args.site_config, args.certificate, args.private_key, args.output_dir)
	except (ManifestError, RenderError, OSError) as error:
		print(f"render-site: {error}", file=sys.stderr)
		return 1
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
