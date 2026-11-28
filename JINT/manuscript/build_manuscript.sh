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
manuscript=$PWD
staging=$(mktemp -d)
trap 'rm -rf -- "$staging"' EXIT
mkdir "$staging/source"
cp main.tex references.bib sn-jnl.cls sn-mathphys-num.bst figures/orcid_mark.png "$staging/source/"
# The flattened submission archive uses only figures referenced by this manuscript.
while IFS= read -r figure; do
  cp "figures/$figure" "$staging/source/"
done < <(sed -n 's/.*\\includegraphics\[[^]]*\]{\([a-zA-Z0-9_]*\.pdf\)}.*/\1/p' main.tex | sort -u)

if [[ -n $field_log ]]; then
  python3 ../analysis/summarize_trials.py "$field_log" \
    --latex-output "$staging/source/formal_trial_rows.tex" \
    --figure-output "$staging/source/formal_trial_outcomes.pdf" "$@"
  cp field_navigation.tex "$staging/source/"
  for sequence in figures/formal_trial_sequence{,_2}.pdf; do
    if [[ -f $sequence ]]; then cp "$sequence" "$staging/source/"; fi
  done
fi

cd "$staging/source"
zip -q "$staging/JINT_draft_source.zip" ./*
unzip -tq "$staging/JINT_draft_source.zip"
unzip -q "$staging/JINT_draft_source.zip" -d "$staging/check"
cd "$staging/check"
latex_options=(-pdf -interaction=nonstopmode -halt-on-error)
latexmk "${latex_options[@]}" main.tex
latexmk "${latex_options[@]}" -jobname=author_review -usepretex='\def\JINTAuthorReview{}' main.tex
cp main.pdf "$manuscript/../JINT_first_draft.pdf"
cp author_review.pdf "$manuscript/../planning/JINT_author_review.pdf"
mv "$staging/JINT_draft_source.zip" "$manuscript/JINT_draft_source.zip"
