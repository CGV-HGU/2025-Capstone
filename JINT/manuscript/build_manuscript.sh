#!/usr/bin/env bash
set -euo pipefail

if [[ ${1-} == --help ]]; then
  echo "Usage: $0 [FIELD_LOG [--configuration ID]]"
  exit 0
fi
field_log=""
if (( $# > 0 )); then
  field_log=$(realpath -e -- "$1")
  shift
fi

cd "$(dirname "${BASH_SOURCE[0]}")"
staging=$(mktemp -d)
trap 'rm -rf -- "$staging"' EXIT
if [[ -n $field_log ]]; then
  python3 ../analysis/summarize_trials.py "$field_log" \
    --latex-output "$staging/formal_trial_rows.tex" \
    --figure-output "$staging/formal_trial_outcomes.pdf" "$@"
  mv "$staging/formal_trial_rows.tex" formal_trial_rows.tex
  mv "$staging/formal_trial_outcomes.pdf" formal_trial_outcomes.pdf
fi

shopt -s nullglob
sources=(main.tex references.bib sn-jnl.cls sn-mathphys-num.bst figures/orcid_mark.png)
for figure in figures/*.pdf; do
  if [[ $figure == figures/reference_sites.pdf ||
        ( $figure == figures/site_views_draft.pdf && -f figures/formal_sites.pdf ) ]]; then
    continue
  fi
  sources+=("$figure")
done
if [[ -f formal_trial_rows.tex ]]; then
  sources+=(formal_trial_rows.tex)
  if grep -Fq '\JINTFormalOutcomesFigure{formal_trial_outcomes.pdf}' formal_trial_rows.tex; then
    sources+=(formal_trial_outcomes.pdf)
  fi
fi

# Rebuild when the optional formal-results file appears or disappears.
latex_options=(-g -pdf -interaction=nonstopmode -halt-on-error -outdir=.build)
latexmk "${latex_options[@]}" main.tex
latexmk "${latex_options[@]}" -jobname=author_review -usepretex='\def\JINTAuthorReview{}' main.tex

zip -qj "$staging/JINT_draft_source.zip" "${sources[@]}"
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
cp .build/author_review.pdf ../planning/JINT_author_review.pdf
mv "$staging/JINT_draft_source.zip" JINT_draft_source.zip
