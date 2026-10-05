import subprocess

tex = r"""\documentclass{ieeeaccess}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{tabularx}
\begin{document}

\begin{table*}[!t]
  \centering
  \footnotesize
  \caption{Comprehensive Comparison Across Robotic Obstacle Avoidance Sensory Modalities}
  \label{tab:sensory_comparison}
  \begin{tabularx}{\textwidth}{@{}p{2.3cm}p{2.5cm}cccccX@{}}
    \toprule
    Modality & Representative Hardware & Approx. Cost (USD) & Payload Weight & Power Draw & Horiz. FOV & Metric Scale? & Primary Vulnerability / Compute Overhead \\
    \midrule
    \textbf{2D Planar LiDAR} & RPLIDAR A2 / UST-10LX & \$300--\$1,800 & 190--400\,g & 4.0--8.0\,W & 270$^\circ$--360$^\circ$ & Direct (ToF) & Glass penetration; waxed floor beam deflections; mechanical motor wear. \\
    \textbf{3D Solid-State LiDAR} & Livox Mid-360 / Ouster & \$800--\$4,000 & 265--450\,g & 6.5--15.0\,W & 360$^\circ \times 59^\circ$ & Direct (ToF) & High hardware cost; heavy 3D pointcloud downsampling compute on edge CPU. \\
    \textbf{RGB-D Camera} & Intel RealSense D435i & \$350--\$500 & 72\,g & 2.5--3.5\,W & 86$^\circ \times 57^\circ$ & Direct (Stereo/IR) & Extreme sunlight / glossy floor IR scattering; limited outdoor range. \\
    \textbf{Monocular Dense MDE} & Standard RGB + DepthAnyV2 & $<$ \$20 & $<$ 30\,g & $<$ 1.0\,W & 70$^\circ$--90$^\circ$ & Relative only & Uncalibrated scale; prohibitive transformer latency ($>170$\,ms). \\
    \textbf{V-LiDAR (Proposed)} & \textbf{Standard RGB + OpenVINO} & \textbf{$<$ \$20} & \textbf{$<$ 30\,g} & \textbf{$<$ 1.0\,W} & \textbf{70.0$^\circ$ (141ch)} & \textbf{Direct (2D LUT)} & \textbf{Near-field blind spot ($<2.18$\,m); lightweight compute (12.9\,ms).} \\
    \bottomrule
  \end{tabularx}
\end{table*}

\EOD
\end{document}
"""

with open('test_table4.tex', 'w', encoding='utf-8') as f:
    f.write(tex)

res = subprocess.run(['pdflatex', '-interaction=nonstopmode', 'test_table4.tex'], capture_output=True, text=True)
errs = [l for l in res.stdout.splitlines() if l.startswith('!')]
print('Errors:', len(errs))
for e in errs:
    print(' ', e)
print('Returncode:', res.returncode)
