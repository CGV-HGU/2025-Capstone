# IJCAS·ICCAS 원고 PDF

[ICCAS-IJCAS-2026](https://github.com/CGV-HGU/ICCAS-IJCAS-2026)의 `main` 커밋 [`078bffc`](https://github.com/CGV-HGU/ICCAS-IJCAS-2026/commit/078bffca970a715b80542284daf65000d18c777d)에 있는 원고를 내용 수정 없이 렌더링했다. PDF 생성일은 2026-11-18이다.

| 원고 | 분량 | PDF |
| --- | ---: | --- |
| IJCAS 저널 원고 | 9쪽 | [IJCAS_Manuscript.pdf](IJCAS_Manuscript.pdf) |
| ICCAS 확장 초록 | 2쪽 | [ICCAS_Extended_Abstract.pdf](ICCAS_Extended_Abstract.pdf) |

두 원고의 제목은 *Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation*이다. 새 실험이나 교정 내용을 반영한 개정본이 아니라, 위 커밋의 원고 내용을 확인하기 위한 PDF다.

원본 소스:

- [IJCAS LaTeX](https://github.com/CGV-HGU/ICCAS-IJCAS-2026/blob/078bffca970a715b80542284daf65000d18c777d/paper/IJCAS/Manuscript%20Template%20-%20LaTeX%20%28February%2019%2C%202026%29.tex)
- [ICCAS LaTeX](https://github.com/CGV-HGU/ICCAS-IJCAS-2026/blob/078bffca970a715b80542284daf65000d18c777d/paper/ICCAS/Paper-Template_ICCAS.tex)

빌드에는 원본 class·그림과 pdfLaTeX(pdfTeX 1.40.22, TeX Live 2022/dev/Debian)를 사용했다. `-no-shell-escape -halt-on-error -interaction=nonstopmode` 옵션으로 각 원고를 3회 컴파일했다. 원본 소스 해시 유지, PDF 전송 해시, 제목·결과 텍스트·참조 및 전체 페이지의 렌더링을 확인했다.
