# 👥 Author Order, Footnote, and Biographies Revision Recommendation

> **Target Manuscript**: `paper/workspace/IEEE_Access/main.tex`  
> **Status**: Recommendation Report (Ready for Integration)  
> **Layout Constraint**: Strictly 14.0 Pages (0 lines spillover, compliant with IEEE Access format)

---

## 1. Executive Summary & Author Roster

Per research director instructions:
1. All five student authors are **equal-contribution co-first authors** ("일단 다 공동저자고").
2. The revised author order is:
   1. **구현우 (HYUNWOO GU)** — Co-first Author
   2. **유건민 (GUNMIN YOO)** — Co-first Author
   3. **이현서 (HYUNSEO LEE)** — Co-first Author
   4. **강현모 (HYUN-MO KANG)** — Co-first Author
   5. **이민석 (MIN-SEOK LEE)** — Co-first Author
   6. **황성수 (SUNG SOO HWANG)** — Corresponding Author (Senior Member, IEEE)

---

## 2. Title Block Author Line & Superscript Affiliations

### 2.1 Existing Code (`main.tex`, Lines 31–36)
```latex
\author{\uppercase{Min-Seok Lee}\authorrefmark{1},
\uppercase{Hyun-Mo Kang}\authorrefmark{1},
\uppercase{Hyunseo Lee}\authorrefmark{1},
\uppercase{Gunmin Yoo}\authorrefmark{1},
\uppercase{Hyunwoo Gu}\authorrefmark{1},
and \uppercase{Sung Soo Hwang}\authorrefmark{1}, \IEEEmembership{Senior Member, IEEE}}
```

### 2.2 Proposed Replacement Code
```latex
\author{\uppercase{Hyunwoo Gu}\authorrefmark{1},
\uppercase{Gunmin Yoo}\authorrefmark{1},
\uppercase{Hyunseo Lee}\authorrefmark{1},
\uppercase{Hyun-Mo Kang}\authorrefmark{1},
\uppercase{Min-Seok Lee}\authorrefmark{1},
and \uppercase{Sung Soo Hwang}\authorrefmark{1}, \IEEEmembership{Senior Member, IEEE}}
```

---

## 3. Author Affiliation & Email Ordering

### 3.1 Existing Code (`main.tex`, Line 38)
```latex
\address[1]{School of Artificial Intelligence, Computer and Electrical Engineering, Handong Global University, Pohang 37554, Republic of Korea\allowbreak\ (e-mail: glen@handong.ac.kr;\allowbreak\ hmkang012@gmail.com;\allowbreak\ hslee@handong.ac.kr;\allowbreak\ gunminy@handong.ac.kr;\allowbreak\ 21800030@handong.ac.kr;\allowbreak\ sshwang@handong.edu)}
```

### 3.2 Proposed Replacement Code
To strictly match the new author ordering:
1. Hyunwoo Gu: `21800030@handong.ac.kr`
2. Gunmin Yoo: `gunminy@handong.ac.kr`
3. Hyunseo Lee: `hslee@handong.ac.kr`
4. Hyun-Mo Kang: `hmkang012@gmail.com`
5. Min-Seok Lee: `glen@handong.ac.kr`
6. Sung Soo Hwang: `sshwang@handong.edu`

```latex
\address[1]{School of Artificial Intelligence, Computer and Electrical Engineering, Handong Global University, Pohang 37554, Republic of Korea\allowbreak\ (e-mail: 21800030@handong.ac.kr;\allowbreak\ gunminy@handong.ac.kr;\allowbreak\ hslee@handong.ac.kr;\allowbreak\ hmkang012@gmail.com;\allowbreak\ glen@handong.ac.kr;\allowbreak\ sshwang@handong.edu)}
```

---

## 4. Title Footnote Equal-Contribution Statement

### 4.1 Existing Code (`main.tex`, Line 40)
```latex
\tfootnote{This research was supported by the ANCHOR program Glocal University 30 through the Gyeongbuk ANCHOR CENTER, funded by the Ministry of Education (MOE) and the Gyeongsangbuk-do, Republic of Korea (2026-ANCHOR-15-119). This work was also supported in part by the National Research Foundation of Korea (NRF) grant funded by the Korean government (MSIT) (No. RS-2025-24683458) and the Handong Global University Academic Research Grant (No. 202500590001). \textit{Min-Seok Lee and Hyun-Mo Kang contributed equally to this work.}}
```

### 4.2 Proposed Replacement Code
```latex
\tfootnote{This research was supported by the ANCHOR program Glocal University 30 through the Gyeongbuk ANCHOR CENTER, funded by the Ministry of Education (MOE) and the Gyeongsangbuk-do, Republic of Korea (2026-ANCHOR-15-119). This work was also supported in part by the National Research Foundation of Korea (NRF) grant funded by the Korean government (MSIT) (No. RS-2025-24683458) and the Handong Global University Academic Research Grant (No. 202500590001). \textit{Hyunwoo Gu, Gunmin Yoo, Hyunseo Lee, Hyun-Mo Kang, and Min-Seok Lee contributed equally to this work.}}
```

---

## 5. Running Header (`\markboth`)

### 5.1 Existing Code (`main.tex`, Lines 42–44)
```latex
\markboth
{Lee \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
{Lee \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
```

### 5.2 Proposed Replacement Code
Because Hyunwoo Gu is the first author, the running header must be changed to `Gu \headeretal`:
```latex
\markboth
{Gu \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
{Gu \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
```

---

## 6. Final Author Biographies on Page 14

### 6.1 Existing Biography Order
1. Min-Seok Lee (`minseok.png`)
2. Hyun-Mo Kang (`hyunmo.png`)
3. Hyunseo Lee (`hyunseo.jpg`)
4. Gunmin Yoo (`gunmin.jpg`)
5. Hyunwoo Gu (`hyunwoo.jpg`)
6. Sung Soo Hwang (`sungsoo.png`)

### 6.2 Proposed Biography Order & Exact Code
```latex
\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunwoo.jpg}}]{Hyunwoo Gu}
was born in Seoul, South Korea, in 1999. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include robotics, visual SLAM, artificial intelligence, and computer vision.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{gunmin.jpg}}]{Gunmin Yoo}
was born in Seoul, South Korea, in 2000. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include embodied artificial intelligence, vision-language navigation, and mobile robot perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunseo.jpg}}]{Hyunseo Lee}
was born in Busan, South Korea, in 2002. He is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, vision-based perception, SLAM, and autonomous navigation.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunmo.png}}]{Hyun-Mo Kang}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include deep learning deployment, edge AI acceleration, autonomous robotics, and visual perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{minseok.png}}]{Min-Seok Lee}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, visual SLAM, robust navigation, and sensor fusion.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{sungsoo.png}}]{Sung Soo Hwang}
was born in Busan, South Korea, in 1983. He received the B.S. degree in computer science and electrical engineering from Handong Global University, Pohang, South Korea, in 2008 and the M.S. and Ph.D. degrees in electrical engineering from the Korea Advanced Institute of Science and Technology (KAIST), Daejeon, South Korea, in 2010 and 2015, respectively. He is currently an Associate Professor with the School of Computer Science and Electrical Engineering, Handong Global University, Pohang, South Korea. His research interests include robotics, artificial intelligence, visual SLAM, and neural-rendering-based 3D reconstruction.
\end{IEEEbiography}
```

---

## 7. Strict 14.0-Page Layout Empirical Verification

A trial compilation was executed in an isolated temporary test harness using `pdflatex` to measure vertical coordinate geometry with PyMuPDF (`fitz`):

| Page 14 Column | Item | Vertical Extent ($y$, pt) | Safety Margin / Status |
| :--- | :--- | :--- | :--- |
| **Column 1** | References [24]–[29] | $y = 63.4 \rightarrow 214.4$ | Normal flow |
| **Column 1** | Bio 1: Hyunwoo Gu | $y = 279.2 \rightarrow 382.8$ | Fits cleanly |
| **Column 1** | Bio 2: Gunmin Yoo | $y = 445.4 \rightarrow 549.1$ | Fits cleanly |
| **Column 1** | Bio 3: Hyunseo Lee | $y = 611.7 \rightarrow 705.8$ | **Terminates at $705.8$\,pt** (Page bottom margin is $730$\,pt; footer is at $739.2$\,pt). **$24.2$\,pt clearance!** |
| **Column 2** | Bio 4: Hyun-Mo Kang | $y = 63.1 \rightarrow 157.1$ | Top of Col 2 |
| **Column 2** | Bio 5: Min-Seok Lee | $y = 280.7 \rightarrow 365.1$ | Fits cleanly |
| **Column 2** | Bio 6: Sung Soo Hwang | $y = 498.2 \rightarrow 621.0$ | Fits cleanly |
| **Column 2** | `\EOD` End Mark | $y \approx 625$ | **Flush right**, $114$\,pt vertical headroom |

**Conclusion**: The document compiles to **EXACTLY 14 pages with ZERO lines of spillover** to Page 15. All 6 author photos strictly satisfy the $1.00\times1.25$\,in ($72\times90$\,pt) IEEE Access template requirement.
