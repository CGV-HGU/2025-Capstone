# JINT

- [JINT 투고 형식 초안 PDF](JINT_first_draft.pdf) · [Springer Nature LaTeX 양식](template/springer-nature-latex-2024-12.zip)
- [원고 소스와 그림](manuscript/) · [소스 ZIP](manuscript/JINT_draft_source.zip)
- [주석이 있는 내부 검토본 PDF](planning/JINT_author_review.pdf) · [투고 전략](planning/2026-11-27_JINT_submission_strategy.md) · [초안 체크리스트](planning/JINT_draft_checklist.md) · [커버레터](planning/cover_letter_draft.md)
- [실험 분석과 재현 방법](analysis/README.md) · [rosbag RGB로 논문 사진 만들기](analysis/README.md#실험-환경-사진과-기존-원고-그림) · [보충 그림 PDF](analysis/evidence/ESM_1.pdf)

`JINT/`에서 `bash manuscript/build_manuscript.sh`를 실행하면 두 PDF와 소스 ZIP을 갱신한다. 새 본 주행 결과는 `bash manuscript/build_manuscript.sh ~/data/jint/field_log.csv`로 반영한다. 같은 조건·배치별 성공 건수로 표와 그래프를 만든다. 결과가 확인되면 초록·결론·해석을 갱신하고 저자 선언을 마무리한다.
