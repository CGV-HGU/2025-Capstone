#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
shopt -s nullglob
sources=(main.tex references.bib sn-jnl.cls sn-mathphys-num.bst figures/*.pdf)
if [[ -f generated/formal_trial_rows.tex ]]; then
  sources+=(generated/formal_trial_rows.tex)
fi

# Rebuild when the optional formal-results file appears or disappears.
latexmk -g -pdf -interaction=nonstopmode -halt-on-error -outdir=.build main.tex

staging=$(mktemp -d)
trap 'rm -rf -- "$staging"' EXIT
zip -q "$staging/JINT_draft_source.zip" "${sources[@]}"
unzip -tq "$staging/JINT_draft_source.zip"
unzip -q "$staging/JINT_draft_source.zip" -d "$staging/check"
(
  cd "$staging/check"
  latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex >compile.log 2>&1 || {
    tail -n 50 compile.log >&2
    exit 1
  }
)

cp .build/main.pdf ../JINT_first_draft.pdf
mv "$staging/JINT_draft_source.zip" JINT_draft_source.zip
