%%
%% $ IJCAS.cls $
%%
%%  written by Karnes Kim, Feb. 2004
%%	modified by CDSL, SNU, July 2011; marked with `2011'
%% --------------------------------------------------------------
%% 
%% It may be distributed and/or modified under the
%% conditions of the LaTeX Project Public License, either version 1.2
%% of this license or (at your option) any later version.
%% The latest version of this license is in
%%    http://www.latex-project.org/lppl.txt
%% and version 1.2 or later is part of all distributions of LaTeX
%% version 1999/12/01 or later.
%% 
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{IJCAS}
              [2017/04/08 v0.04 LaTeX class for IJCAS]
              
\usepackage{comment}

%% BASIC PARAMETERS SETTING UP
\@twosidetrue
\@mparswitchfalse
\@twocolumntrue
\newif\if@cmfont
\@cmfontfalse
\newif\if@draft
\newif\if@sloppy
\@draftfalse
\DeclareOption{draft}{\setlength\overfullrule{5pt}\@drafttrue}
\if@compatibility\else
\DeclareOption{final}{\setlength\overfullrule{0pt}\@draftfalse}
\fi
\DeclareOption{cm}{\@cmfonttrue}
\DeclareOption{sloppy}{\@sloppytrue}
\DeclareOption{leqno}{\input{leqno.clo}}
\DeclareOption{fleqn}{\input{fleqn.clo}}
\ExecuteOptions{final}
\ProcessOptions
\RequirePackage[T1]{fontenc}
\RequirePackage{fancyhdr}
\RequirePackage{ifthen}
\RequirePackage{lastpage}
\RequirePackage{cite}
\RequirePackage{picins}
\RequirePackage{tabu}
\RequirePackage{booktabs}
\RequirePackage{color}
\RequirePackage{float}%% for definition of foot* environment. NEVER REMOVE!!
%\RequirePackage{dblfloatfix}
\RequirePackage[fleqn]{amsmath}
\RequirePackage{amsfonts, amssymb, bm, mathrsfs}
\RequirePackage{multicol}
\PassOptionsToPackage{hyphens}{url}
\RequirePackage[colorlinks,hyperindex,breaklinks, hyperfootnotes=false]{hyperref}
\hypersetup{citecolor=blue, linkcolor=black, urlcolor=black}
\RequirePackage[justification=justified,singlelinecheck=false]{caption}
\usepackage{stfloats}



%% DOCUMENT FONT: if [cm] options is given, we will use
%% CM font, if not, mathptmx-like TIMES fonts used.
\if@cmfont
\AtBeginDocument{%
  \label{FirstPage}%% Mark the first page for checking the first page number
  }
\else
\@ifpackageloaded{mathptmx}{}{%
\def\rmdefault{ptm}
\DeclareSymbolFont{operators}   {OT1}{ztmcm}{m}{n}
\DeclareSymbolFont{letters}     {OML}{ztmcm}{m}{it}
\DeclareSymbolFont{symbols}     {OMS}{ztmcm}{m}{n}
\DeclareSymbolFont{largesymbols}{OMX}{ztmcm}{m}{n}
\DeclareSymbolFont{bold}        {OT1}{ptm}{bx}{n}
\DeclareSymbolFont{italic}      {OT1}{ptm}{m}{it}
\@ifundefined{mathbf}{}{\DeclareMathAlphabet{\mathbf}{OT1}{ptm}{bx}{n}}
\@ifundefined{mathit}{}{\DeclareMathAlphabet{\mathit}{OT1}{ptm}{m}{it}}
\DeclareMathAlphabet{\mathcal}{OMS}{cmsy}{m}{n}
\DeclareMathSymbol{\omicron}{0}{operators}{`\o}
\renewcommand{\jmath}{%
  \PackageError{IJCAS}
  {The symbols \protect\jmath, \protect\amalg\space and
  \protect\coprod\MessageBreak
  are not available with this package along with mathptmx}
  {Type \space <return> \space to proceed;
  your command will be ignored.}}
\let\amalg=\jmath
\let\coprod=\jmath
\thinmuskip=2mu
\medmuskip=2.5mu plus 1mu minus 1mu
\thickmuskip=4mu plus 1.5mu minus 1mu
\DeclareRobustCommand\hbar{{%
 \dimen@.03em%
 \dimen@ii.06em%
 \def\@tempa##1##2{{%
   \lower##1\dimen@\rlap{\kern##1\dimen@ii\the##2 0\char22}}}%
 \mathchoice\@tempa\@ne\textfont
            \@tempa\@ne\textfont
            \@tempa\defaultscriptratio\scriptfont
            \@tempa\defaultscriptscriptratio\scriptscriptfont
  h}}
\def\boldmath{%
   \PackageWarning%
   {mathptmx}%
   {There are no bold math fonts}%
   \global\let\boldmath=\relax
}

\AtBeginDocument{%
  \label{FirstPage}%% Mark the first page for checking the first page number
  \DeclareFontFamily{U}{msa}{}%
  \DeclareFontShape{U}{msa}{m}{n}{<->msam10}{}%
  \DeclareFontFamily{U}{msb}{}%
  \DeclareFontShape{U}{msb}{m}{n}{<->msbm10}{}%
  \DeclareFontFamily{U}{euf}{}%
  \DeclareFontShape{U}{euf}{m}{n}{<-6>eufm5<6-8>eufm7<8->eufm10}{}%
  \DeclareFontShape{U}{euf}{b}{n}{<-6>eufb5<6-8>eufb7<8->eufb10}{}%
  \@ifpackageloaded{amsfonts}{\let\hbar\s@vedhbar}{}
  \@ifpackageloaded{amsmath}{}{%
    \newdimen\big@size
    \addto@hook\every@math@size{\setbox\z@\vbox{\hbox{$($}\kern\z@}%
       \global\big@size 1.2\ht\z@}
    \def\bBigg@#1#2{%
       {\hbox{$\left#2\vcenter to#1\big@size{}\right.\n@space$}}}
    \def\big{\bBigg@\@ne}
    \def\Big{\bBigg@{1.5}}
    \def\bigg{\bBigg@\tw@}
    \def\Bigg{\bBigg@{2.5}}
  }%
 }%
}
\fi
%%%
%%% size definitions
%%%
\def\@vpt{5}
\def\@vipt{6}
\def\@viipt{7}
\def\@viiipt{8}
\def\@ninesize{9}
\def\@ixpt{9.5}
\def\@xpt{10}% 10.5 -> 10 on 2011
\def\@elevensize{10.0}% 11.0 -> 10.0 on 2011
\def\@xipt{11.25}
\def\@xiipt{12.24}
\def\@xivpt{15}
\def\@xviipt{17.28}
\def\@xxpt{20.74}
\def\@xxvpt{24.88}
\renewcommand\normalsize{%
   \@setfontsize\normalsize\@xpt\@xiipt
   \abovedisplayskip 10\p@ \@plus2\p@ \@minus5\p@
   \abovedisplayshortskip \z@ \@plus3\p@
   \belowdisplayshortskip 6\p@ \@plus3\p@ \@minus3\p@
   \belowdisplayskip \abovedisplayskip
   \let\@listi\@listI}
\normalsize
\newcommand\small{%
   \@setfontsize\small\@ixpt{11}%
   \abovedisplayskip 8.5\p@ \@plus3\p@ \@minus4\p@
   \abovedisplayshortskip \z@ \@plus2\p@
   \belowdisplayshortskip 4\p@ \@plus2\p@ \@minus2\p@
   \def\@listi{\leftmargin\leftmargini
               \topsep 4\p@ \@plus2\p@ \@minus2\p@
               \parsep 2\p@ \@plus\p@ \@minus\p@
               \itemsep \parsep}%
   \belowdisplayskip \abovedisplayskip
}
\newcommand\footnotesize{%
%   \@setfontsize\footnotesize\@ninesize{9.5}%
   \@setfontsize\footnotesize\@ninesize{9}%
   \abovedisplayskip 6\p@ \@plus2\p@ \@minus4\p@
   \abovedisplayshortskip \z@ \@plus\p@
   \belowdisplayshortskip 3\p@ \@plus\p@ \@minus2\p@
   \def\@listi{\leftmargin\leftmargini
               \topsep 3\p@ \@plus\p@ \@minus\p@
               \parsep 2\p@ \@plus\p@ \@minus\p@
               \itemsep \parsep}%
   \belowdisplayskip \abovedisplayskip
}
\newcommand\scriptsize{\@setfontsize\scriptsize\@viipt\@viiipt}
\newcommand\tiny{\@setfontsize\tiny\@vpt\@vipt}
\newcommand\ninesize{\@setfontsize\ninesize\@ninesize{9.5}}
\newcommand\elevensize{\@setfontsize\elevensize\@elevensize{11.5}}
\newcommand\large{\@setfontsize\large\@xiipt{14}}
\newcommand\Large{\@setfontsize\Large\@xivpt{18}}
\newcommand\LARGE{\@setfontsize\LARGE\@xviipt{22}}
\newcommand\huge{\@setfontsize\huge\@xxpt{25}}
\newcommand\Huge{\@setfontsize\Huge\@xxvpt{30}}
\setlength\parindent{1em}
\setlength\smallskipamount{3\p@ \@plus 1\p@ \@minus 1\p@}
\setlength\medskipamount{6\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\bigskipamount{12\p@ \@plus 4\p@ \@minus 4\p@}
\setlength\headheight{12\p@}
\setlength\headsep   {25\p@}
\setlength\topskip   {10\p@}
\setlength\footskip{30\p@}
\if@compatibility \setlength\maxdepth{4\p@} \else
\setlength\maxdepth{.5\topskip} \fi
%%%%% *
\setlength{\skip\footins}{9\p@ \@plus 4\p@ \@minus 2\p@}
\setlength\floatsep    {12\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\textfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\intextsep   {12\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\dblfloatsep    {12\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\dbltextfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}
\setlength\@fptop{0\p@ \@plus 1fil}
\setlength\@fpsep{8\p@ \@plus 2fil}
\setlength\@fpbot{0\p@ \@plus 1fil}
\setlength\@dblfptop{0\p@ \@plus 1fil}
\setlength\@dblfpsep{8\p@ \@plus 2fil}
\setlength\@dblfpbot{0\p@ \@plus 1fil}
\setlength\partopsep{2\p@ \@plus 1\p@ \@minus 1\p@}
\def\@listi{\leftmargin\leftmargini
            \parsep 2\p@ \@plus2\p@ \@minus\p@
            \topsep 6\p@ \@plus2\p@ \@minus4\p@
            \itemsep0\p@ \@plus1\p@ \@minus\p@}
\let\@listI\@listi
\@listi
\def\@listii {\leftmargin\leftmarginii
              \labelwidth\leftmarginii
              \advance\labelwidth-\labelsep
              \topsep    4\p@ \@plus2\p@ \@minus\p@
              \parsep    2\p@ \@plus\p@  \@minus\p@
              \itemsep   0\p@}
\def\@listiii{\leftmargin\leftmarginiii
              \labelwidth\leftmarginiii
              \advance\labelwidth-\labelsep
              \topsep    2\p@ \@plus\p@\@minus\p@
              \parsep    \z@
              \partopsep \p@ \@plus\z@ \@minus\p@
              \itemsep   0\p@}
\def\@listiv {\leftmargin\leftmarginiv
              \labelwidth\leftmarginiv
              \advance\labelwidth-\labelsep}
\def\@listv  {\leftmargin\leftmarginv
              \labelwidth\leftmarginv
              \advance\labelwidth-\labelsep}
\def\@listvi {\leftmargin\leftmarginvi
              \labelwidth\leftmarginvi
              \advance\labelwidth-\labelsep}
%%%
\setlength\lineskip{1\p@}
\setlength\normallineskip{1\p@}
\renewcommand\baselinestretch{}
\setlength\parskip{0\p@ \@plus \p@}
\@lowpenalty   51
\@medpenalty  151
\@highpenalty 301
\setcounter{topnumber}{2}
\renewcommand\topfraction{.7}
\setcounter{bottomnumber}{1}
\renewcommand\bottomfraction{.3}
\setcounter{totalnumber}{3}
\renewcommand\textfraction{.2}
\renewcommand\floatpagefraction{.75}
\setcounter{dbltopnumber}{2}
\renewcommand\dbltopfraction{.8}
\renewcommand\dblfloatpagefraction{.75}

%%% PAGESTYLE
\def\ps@IJCASheadings{%
      \let\@oddfoot\@empty\let\@evenfoot\@empty
	  \def\@oddhead{\sf\small\@title\hfil\thepage} % replaced on 2015
	  \def\@evenhead{\sf\small\journal@title\hfil\thepage} % replaced on 2015
      \let\@mkboth\markboth
    \def\sectionmark##1{%
      \markboth {\MakeUppercase{%
        \ifnum \c@secnumdepth >\z@
          \thesection\quad
        \fi
        ##1}}{}}%
    \def\subsectionmark##1{%
      \markright {%
        \ifnum \c@secnumdepth >\@ne
          \thesubsection\quad
        \fi
        ##1}}}

%%% MAKETITLE
\newcommand\maketitle{\par
  \begingroup
	\renewcommand\thefootnote{}%
    \def\@makefnmark{\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
    \long\def\@makefntext##1{\parindent 1em\noindent
            \hb@xt@1.5em{%
                \hss\@textsuperscript{\normalfont\@thefnmark}}##1}%
      \ifnum \col@number=\@ne
        \@maketitle
      \else
   \twocolumn[\@maketitle]%
      \fi
    \thispagestyle{plain}\@thanks
  \endgroup
  \setcounter{footnote}{0}%
  \global\let\thanks\relax
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@date\@empty
  \global\let\@title\@empty
  \global\let\title\relax
  \global\let\author\relax
  \global\let\date\relax
  \global\let\and\relax
}

\def\@maketitle{%
  \newpage
  \null
  \vskip 1.0em%
  \begin{center}%
    {\minipage[b]{\textwidth}{\fontsize{15.5pt}{18pt}\selectfont{\textcolor[RGB]{70,126,166}{\bf\@title}} \par}\endminipage}%
    \vskip .5em%
    {%
      \lineskip .5em%
      \minipage[b]{\textwidth}{
        {\elevensize\bf\@author}%
      }%
      \endminipage}
  \vskip -1pc
  \hskip 0.1pc\hbox to \textwidth{\leaders\hrule\hfill}%
  \vskip 1em%
  \abstract@produce
  \vskip 1.0em%
  \keyword@produce
  \vskip -1pc%\vskip -0.5 em%
  \hskip 0.1pc\hbox to \textwidth{\leaders\hrule\hfill}%
  \end{center}%
	\vskip 1pc
	}
%
%%%
\newbox\abstractbox
\newenvironment{abstract}{%
   \lrbox\abstractbox
     \minipage[b]{.95\textwidth}%
     \bfseries\normalsize\abstractname~%
	 \normalfont\normalsize\ignorespaces%
  }{%
   \endminipage\endlrbox}
\def\abstract@produce{%
   \leavevmode\box\abstractbox\par
}
%%%
\newbox\keywordbox
\newenvironment{keywords}{%
   \lrbox\keywordbox
     \minipage[b]{.95\textwidth}%
     \bfseries\normalsize\keywordname~%
	 \normalfont\normalsize\ignorespaces%
  }{%
   \endminipage\endlrbox}
\def\keyword@produce{%
   \leavevmode\box\keywordbox\par
}
%%%
\newbox\footnotebox
\newenvironment{makefoot}{%
   \lrbox\footnotebox
     \minipage{.95\textwidth}%
     \normalfont\fontsize{8.5pt}{8.5pt}\ignorespaces%
  }{%
   \endminipage\endlrbox}
\def\footnote@produce{%
   \leavevmode\box\footnotebox\par
}

%%% SECTION commands:
%%% \section and \subsection are effective, but other commands will be preserved
%%% for compatibility.
\newcommand*{\@seccntformat@section}[1]{\csname the#1\endcsname.\quad}
\newcommand*{\@seccntformat@subsection}[1]{\csname the#1\endcsname.\quad}
\newcommand*{\@seccntformat@subsubsection}[1]{\csname the#1\endcsname\quad}
\let\@@seccntformat\@seccntformat
\renewcommand*{\@seccntformat}[1]{%
  \expandafter\ifx\csname @seccntformat@#1\endcsname\relax
    \expandafter\@@seccntformat
  \else
    \expandafter
      \csname @seccntformat@#1\expandafter\endcsname
  \fi
    {#1}%
}



\setcounter {secnumdepth}{5}
\newcounter {section}
\newcounter {subsection}[section]
\newcounter {subsubsection}[subsection]
\newcounter {paragraph}[subsubsection]
\newcounter {subparagraph}[paragraph]
\renewcommand\thesection {\normalsize\@arabic\c@section}
\renewcommand\thesubsection   {\thesection.\@arabic\c@subsection}
\renewcommand\thesubsubsection{\thesubsection.\@arabic\c@subsubsection}
\renewcommand\theparagraph    {\thesubsubsection.\@arabic\c@paragraph}
\renewcommand\thesubparagraph {\theparagraph.\@arabic\c@subparagraph}
\newcommand\MakeUppercaseBlue[1]{
\textcolor[RGB]{70,126,166}{\bfseries\MakeUppercase{#1}}
}

\newcommand\section{\@startsection {section}{1}{\z@}%
                   {-3.5ex \@plus -.5ex \@minus -.2ex}%
                   {2.0ex \@plus.2ex}%
                   {\centering\normalfont\elevensize\MakeUppercaseBlue}}
\newcommand\subsection{\@startsection{subsection}{2}{\z@}%
                      {-2.8ex\@plus -.5ex \@minus -.2ex}%
                      {0.5ex \@plus .1ex}%
                      {\noindent\normalfont\fontsize{10.5pt}{10.5pt}\selectfont\textcolor[RGB]{70,126,166}}}
\newcommand\subsubsection{\@startsection{subsubsection}{3}{\z@}%
                         {-2.8ex\@plus -.5ex \@minus -.2ex}%
                         {0.5ex \@plus .1ex}%
                         {\noindent\normalfont\fontsize{10.5pt}{10.5pt}\selectfont\textcolor[RGB]{70,126,166}}}
\newcommand\paragraph{\@startsection{paragraph}{4}{\z@}%
                     {1.5ex \@plus1ex \@minus.2ex}%
                     {-1em}%
                     {\normalfont\normalsize\bfseries}}
\newcommand\subparagraph{\@startsection{subparagraph}{5}{\parindent}%
                        {3.25ex \@plus1ex \@minus .2ex}%
                        {-1em}%
                        {\normalfont\normalsize\bfseries}}

\leftmargin  \leftmargini
\setlength\leftmargini {1.8em}
\setlength\leftmarginii  {2.2em}
\setlength\leftmarginiii {1.87em}
\setlength\leftmarginiv  {1.7em}
\setlength\leftmarginv  {.5em}
\setlength\leftmarginvi {.5em}
\setlength  \labelsep  {.5em}
\setlength  \labelwidth{\leftmargini}
\addtolength\labelwidth{-\labelsep}
\@beginparpenalty -\@lowpenalty
\@endparpenalty   -\@lowpenalty
\@itempenalty     -\@lowpenalty
\renewcommand\theenumi{\@arabic\c@enumi}
\renewcommand\theenumii{\@alph\c@enumii}
\renewcommand\theenumiii{\@roman\c@enumiii}
\renewcommand\theenumiv{\@Alph\c@enumiv}
\newcommand\labelenumi{\theenumi.}
\newcommand\labelenumii{(\theenumii)}
\newcommand\labelenumiii{\theenumiii.}
\newcommand\labelenumiv{\theenumiv.}
\renewcommand\p@enumii{\theenumi}
\renewcommand\p@enumiii{\theenumi(\theenumii)}
\renewcommand\p@enumiv{\p@enumiii\theenumiii}
\newcommand\labelitemi{\textbullet}
\newcommand\labelitemii{\normalfont\bfseries \textendash}
\newcommand\labelitemiii{\textasteriskcentered}
\newcommand\labelitemiv{\textperiodcentered}
\newenvironment{description}
               {\list{}{\labelwidth\z@ \itemindent-\leftmargin
                        \let\makelabel\descriptionlabel}}
               {\endlist}
\newcommand*\descriptionlabel[1]{\hspace\labelsep
                                \normalfont\bfseries #1}
\newenvironment{quotation}
               {\list{}{\listparindent 1.5em%
                        \itemindent    \listparindent
                        \rightmargin   \leftmargin
                        \parsep        \z@ \@plus\p@}%
                \item\relax}
               {\endlist}
\newenvironment{quote}
               {\list{}{\rightmargin\leftmargin}%
                \item\relax}
               {\endlist}
\newcounter{appendixcnt}
\setcounter{appendixcnt}{0}
\newcommand\appendix{\par\medskip
  \stepcounter{appendixcnt}%
  \section*{\MakeUppercase\appendixname~\Alph{appendixcnt}}\par
  \setcounter{section}{0}%
  \setcounter{subsection}{0}%
  \setcounter{equation}{0}%
  \def\theequation{\Alph{appendixcnt}.\arabic{equation}}%
  \gdef\thesubsection{\Alph{appendixcnt}.\arabic{subsection}}%
}

\setlength\arraycolsep{5\p@}
\setlength\tabcolsep{6\p@}
\setlength\arrayrulewidth{.4\p@}
\setlength\doublerulesep{2\p@}
\setlength\tabbingsep{\labelsep}
\skip\@mpfootins = \skip\footins
\setlength\fboxsep{3\p@}
\setlength\fboxrule{.4\p@}
\renewcommand \theequation {\@arabic\c@equation}
\newcounter{figure}
\renewcommand \thefigure {\@arabic\c@figure}
\def\fps@figure{tbp}
\def\ftype@figure{1}
\def\ext@figure{lof}
\def\fnum@figure{\figurename\nobreakspace\textcolor[RGB]{70,126,166}{\thefigure}}
\newenvironment{figure}
               {\@float{figure}}
               {\end@float}
\newenvironment{figure*}
               {\@dblfloat{figure}}
               {\end@dblfloat}
\newcounter{table}
\renewcommand\thetable{\@arabic\c@table}
\def\fps@table{tbp}
\def\ftype@table{2}
\def\ext@table{lot}
\def\fnum@table{\tablename\nobreakspace\textcolor[RGB]{70,126,166}{\thetable}}
\newenvironment{table}
               {\@float{table}}
               {\end@float}
\newenvironment{table*}
               {\@dblfloat{table}}
               {\end@dblfloat}
\setlength\abovecaptionskip{10\p@}
\setlength\belowcaptionskip{5\p@}
\begin{comment}
\long\def\@makecaption#1#2{%
  \vskip\abovecaptionskip
  \sbox\@tempboxa{#1\textcolor[RGB]{70,126,166}{.} #2}
  \ifdim \wd\@tempboxa >\hsize
    #1\textcolor[RGB]{70,126,166}{.} #2\par
  \else
    \global \@minipagefalse
    \hb@xt@\hsize{\hfil\box\@tempboxa\hfil}%
  \fi
  \vskip\belowcaptionskip}
\end{comment}
\DeclareOldFontCommand{\rm}{\normalfont\rmfamily}{\mathrm}
\DeclareOldFontCommand{\sf}{\normalfont\sffamily}{\mathsf}
\DeclareOldFontCommand{\tt}{\normalfont\ttfamily}{\mathtt}
\DeclareOldFontCommand{\bf}{\normalfont\bfseries}{\mathbf}
\DeclareOldFontCommand{\it}{\normalfont\itshape}{\mathit}
\DeclareOldFontCommand{\sl}{\normalfont\slshape}{\@nomath\sl}
\DeclareOldFontCommand{\sc}{\normalfont\scshape}{\@nomath\sc}
\DeclareRobustCommand*\cal{\@fontswitch\relax\mathcal}
\DeclareRobustCommand*\mit{\@fontswitch\relax\mathnormal}
\newcommand\@pnumwidth{1.55em}
\newcommand\@tocrmarg{2.55em}
\newcommand\@dotsep{4.5}
\newdimen\bibindent
\setlength\bibindent{1.5em}
\newenvironment{thebibliography}[1]
     {%
      \renewcommand\section{\@startsection {section}{1}{\z@}%
                       {-3.5ex \@plus -1ex \@minus -.2ex}%
                       {2.3ex \@plus.2ex}%
                       {\centering\normalfont\elevensize\bfseries\MakeUppercase}}
      \section*{\refname}%
      \fontsize{9pt}{11pt}\selectfont
      \@mkboth{\MakeUppercase\refname}{\MakeUppercase\refname}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \setlength\itemsep{\parsep}
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}
\newcommand\newblock{\hskip .11em\@plus.33em\@minus.07em}
\let\@openbib@code\@empty
\newcommand\@idxitem{\par\hangindent 40\p@}
\newcommand\subitem{\@idxitem \hspace*{20\p@}}
\newcommand\subsubitem{\@idxitem \hspace*{30\p@}}
\newcommand\indexspace{\par \vskip 10\p@ \@plus5\p@ \@minus3\p@\relax}
\renewcommand\footnoterule{%
  \kern-3\p@
  \hrule\@width.4\columnwidth
  \kern2.6\p@}
\newcommand\@makefntext[1]{%
    \parindent 1em%
    \noindent
    \hb@xt@1.8em{\hss\@makefnmark}#1}
\newcommand\refname{\textcolor[RGB]{70,126,166}{References}}
\newcommand\figurename{\textcolor[RGB]{70,126,166}{Fig.}} 
\newcommand\tablename{\textcolor[RGB]{70,126,166}{Table}}  
\newcommand\appendixname{Appendix}
\newcommand\abstractname{\textcolor[RGB]{227,108,10}{Abstract:}}
\newcommand\keywordname{\textcolor[RGB]{227,108,10}{Keywords:}}
\def\today{\ifcase\month\or
  January\or February\or March\or April\or May\or June\or
  July\or August\or September\or October\or November\or December\fi
  \space\number\day, \number\year}
\setlength\columnsep{12\p@}
\setlength\columnseprule{0\p@}
\pagestyle{plain}
\pagenumbering{arabic}
\raggedbottom
\onecolumn
\if@sloppy
   \sloppy
\else
\fi
\flushbottom
\oddsidemargin=-15pt
\evensidemargin\oddsidemargin
\headheight=12pt
\textheight=672pt%682pt
\marginparsep=10pt
\footskip=30pt
\hoffset=0pt
\paperwidth=597pt
\topmargin=-52pt
\headsep=15pt%25pt
\textwidth=483pt
\marginparwidth=48pt
\marginparpush=5pt
\voffset=0pt
\paperheight=796pt
\columnsep=2.14em % added on 2011



\RequirePackage{amsthm,amssymb}
\thm@style{plain}
\newtoks\thm@bodyfont  \thm@bodyfont{\normalfont}
\newtoks\thm@headfont  \thm@headfont{\bfseries}
\newtoks\thm@notefont  \thm@notefont{}
\newtoks\thm@headpunct \thm@headpunct{.}
\newskip\thm@preskip \newskip\thm@postskip
\def\thm@space@setup{%
  \thm@preskip=\topsep \thm@postskip=\thm@preskip
} 
\def\th@plain{%
  \let\thm@indent\indent % no indent
  \thm@headpunct{:}% add period after heading
  \normalfont% body font
} 
\def\@endtheorem{\endtrivlist\@endpefalse} 

%%% borrowed from amsart
\def\qed{\ifhmode\unskip\nobreak\fi\hfill
  \ifmmode\square\else$\m@th\square$\fi}

\renewenvironment{proof}[1][\proofname]{\par
  \normalfont
  \topsep6\p@\@plus6\p@ \trivlist
  \item[\hskip\labelsep\indent
  \bfseries
    #1\@addpunct{:}]\ignorespaces
}{%
  \qed\endtrivlist
}

%%% BIOGRAPHY : borrowed from IEEEtran and modified
\newif\if@biographyTOCentrynotmade
\global\@biographyTOCentrynotmadetrue

\newdimen\@tmpdimenA
\newdimen\@tmpdimenB 
\newcount\@tmpcountA
\newcount\@tmpcountB 
\begingroup
\catcode`\Q=3
\long\gdef\@ifmtarg#1{\@xifmtarg#1QQ\@secondoftwo\@firstoftwo\@nil}
\long\gdef\@xifmtarg#1#2Q#3#4#5\@nil{#4}
\endgroup 
\def\@needspace#1#2{\penalty-100\begingroup%shield temp variable
\@tmpdimenB\pagegoal\advance\@tmpdimenB-\pagetotal% space left
\ifdim #1>\@tmpdimenB\relax% not enough space left
\ifdim\@tmpdimenB>\z@\relax #2\fi%
\newpage%
\fi\endgroup} 

\newcounter{biography}
\setcounter{biography}{0}

\def\@photowidth{25mm}   % width of the biography photo area
\def\@photodepth{30mm}   % depth (height) of the biography photo area
\def\@hangwidth{27.5mm}  % width cleared for the biography photo area
\def\@hangdepth{31.5mm}  % depth cleared for the biography photo area
\def\@skipN{1.5\baselineskip}% nominal value of the vskip above the biography

\if@draft
\RequirePackage[draft]{graphicx}
\else
\RequirePackage{graphicx}
\fi
\begin{comment}
\newcommand\biography[3]{
\noindent 
\parpic[l]{
\includegraphics[width=25mm,height=30mm,clip,keepaspectratio]{#1}
}\vskip 0.5pc
{\noindent\fontsize{9pt}{10pt}\selectfont\textcolor[RGB]{70,126,166}{\textbf{#2}}}
 {\fontsize{9pt}{10pt}\selectfont{#3}}
}
\end{comment}
%\begin{comment}
\newcommand\biography[3]%
{\def\temp{#1}\ifx\temp\empty
{\noindent\fontsize{9pt}{10pt}\selectfont\textcolor[RGB]{70,126,166}{\textbf{#2}}}
 {\fontsize{9pt}{10pt}\selectfont{#3}}
\else
\begin{@biography}[{\includegraphics[width=25mm,height=30mm,clip,keepaspectratio]{#1}}]{\fontsize{9pt}{10pt}\selectfont\textcolor[RGB]{70,126,166}{\textbf{#2}}}
 {\fontsize{9pt}{10pt}\selectfont{#3}}
 \end{@biography}
\fi
}

\newenvironment{@biography}[2][]{\normalfont\small%
\unitlength 1in\parskip=0pt\par\parindent 1em\interlinepenalty500%
\@tmpdimenA=\@hangdepth%
\advance\@tmpdimenA by \@skipN%
\advance\@tmpdimenA by 1\baselineskip%
\@needspace{\@tmpdimenA}{\relax}%
\vskip \@skipN plus 1fil minus 0\baselineskip%
\def\@tempbiographybox{{\setlength{\fboxsep}{0pt}\framebox{\begin{minipage}[b][\@photodepth][c]{\@photowidth}\centering PLACE\\ PHOTO\\ HERE \end{minipage}}}}%
\@ifmtarg{#1}{\relax}{\def\@tempbiographybox{\mbox{\begin{minipage}[b][\@photodepth][c]{\@photowidth}%
\centering%
#1%
\end{minipage}}}}% end if optional argument supplied
\if@biographyTOCentrynotmade%
\setcounter{biography}{-1}%
\refstepcounter{biography}%
\addcontentsline{toc}{section}{Biographies}%
\global\@biographyTOCentrynotmadefalse%
\fi%
\refstepcounter{biography}%
\addcontentsline{toc}{subsection}{#2}%
\let\@ORGparCMD=\par% 
\edef\par{\hfil\break\indent}%
\settoheight{\@tmpdimenA}{\@tempbiographybox}%
\@tmpdimenB=\@hangdepth%
\@tmpcountA=\@tmpdimenB% countA has the hang depth
\divide\@tmpcountA by \baselineskip%
\advance\@tmpcountA by 1%
\hangindent\@hangwidth%
\hangafter-\@tmpcountA%
\settoheight{\@tmpdimenB}{\mbox{T}}%
\noindent\makebox[0pt][l]{\hspace{-\@hangwidth}\raisebox{\@tmpdimenB}[0pt][0pt]{\raisebox{-\@photodepth}[0pt][0pt]{\@tempbiographybox}}}%
\noindent\textbf{#2\ }\ignorespaces}
{
\relax\let\par=\@ORGparCMD\par%
 \ifnum \prevgraf <\@tmpcountA\relax%
    \advance\@tmpcountA by -\prevgraf%
    \advance\@tmpcountA by -1\relax%
    \@tmpdimenA=\baselineskip%
    \multiply\@tmpdimenA by \@tmpcountA%
    \noindent\rule{0pt}{\@tmpdimenA}%
\fi%
\par\normalfont
}
%\end{comment}

\let\lastpagebalance\balance

\newcommand\journaltitle[1]%
	{\gdef\journal@title{#1}}
\newcommand\journalnumber[1]%
	{\gdef\journal@number{#1}}
\newcommand\journalvolumn[1]%
	{\gdef\journal@volumn{#1}}
\newcommand\journalyear[1]%
	{\gdef\journal@year{#1}}

\journaltitle{International Journal of Control, Automation, and Systems}
\journalvolumn{}
\journalnumber{}
\journalyear{}

\newcommand\setarticlestartpagenumber[1]%
	{
	\setcounter{page}{#1}
	\gdef\startingpage{#1}
	}

\newenvironment{reference}
	{\begin{thebibliography}{00}}
	{\end{thebibliography}}

\newcommand\clearafterbiography{%
\PublisherNote
	\vspace*{\stretch{1}}%
}

\newcommand\PublisherNote{
\vskip 1pc
\noindent
\fontsize{9pt}{11pt}\selectfont\textbf{Publisher's Note} Springer Nature remains neutral with regard to jurisdictional claims in published maps and institutional affiliations.
}


%runningtitle{year}{authors}{title}{DOI1}{DOI2}{DOI3}
\newcommand{\runningtitle}[6]{
\pagestyle{fancy}
\fancyhead{}
%\fancyhead[LO]{\footnotesize\textsf{#3}\hfill\thepage}
\fancyhead[RO]{\footnotesize\thepage}
\fancyhead[CO]{\footnotesize\textsf{#3}}% 
\fancyhead[LE]{\footnotesize\thepage}
\fancyhead[CE]{\footnotesize\textsf{#2}}
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}
\fancyfoot{}
%\ifthenelse{\equal{#1}{}}{\setcounter{page}{1}}{\setcounter{page}{#1}}
\thispagestyle{plain}
\fancypagestyle{plain}{
%
%\marginsize{15mm}{15mm}{10mm}{30mm}
%\
\fancyhf{}
\fancyhead[L]{
\begin{flushleft}
%\begin{tabular}{l}
\footnotesize%\textsf
{\journal@title ~\journal@volumn(\journal@number) (\journal@year) \pageref{FirstPage}-\pageref{LastPage}}\\%\startingpage
\footnotesize%\textsf
{http://dx.doi.org/10.1007/s12555-#4-#5-#6}%014-0328-10}
%\end{tabular}
\end{flushleft}
%DOI 10.1007/s12555-014-0328-10
}
\fancyhead[R]{
\begin{flushright}
\footnotesize{ISSN:1598-6446 eISSN:2005-4092}\\
\footnotesize{\urlstyle{rm}\url{http://www.springer.com/12555}}
\end{flushright}
}

\fancyfoot[R]{\vskip -1pc\footnotesize\copyright ICROS, KIEE and Springer #1}
%\fancyfoot[L]{\includegraphics[width=2cm]{springer_mark.jpg}}

\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}
}
}



\newcommand\blfootnote[1]{%
\renewcommand*\footnoterule{}
\interfootnotelinepenalty=10000
  \begingroup
  \renewcommand\thefootnote{}\footnote{#1}%
  \addtocounter{footnote}{-1}%
  \endgroup
}


\newfloat{foot}{b}{ft}


\newcommand{\makeAuthorInformation}[1]{
\begin{authorInforms}
#1
\end{authorInforms}
\begin{foot*}[!b]
\vskip -0.35pc
\begin{center}
\rule[2pt]{\textwidth}{0.5pt}\\
\vskip 0.3pc
\aif@produce
\vskip 0.3pc
\rule[5pt]{\textwidth}{0.5pt}
\end{center}
\vskip -1pc
\end{foot*}
}

\newbox\aifbox

\newenvironment{authorInforms}{%
   \lrbox\aifbox
     \hskip 0.2pc\minipage{.99\textwidth}%
   	 \fontsize{8.5pt}{10pt}\selectfont\ignorespaces%
  }{%
   \endminipage\endlrbox}
\def\aif@produce{%
   \leavevmode\box\aifbox\par
}

\twocolumn

%% Theorem, Lemma, Remarks, etc.
\newtheoremstyle{cited}%
  {3pt}% (space above)
  {3pt}% (space below)
  {}% (body font)
  {0.8pc}%{1pc}% (indent amount)
  {\bfseries}% {theorem head font}
  {\textbf{:}}% {punctuation after theorem head}
  {.5em}% {space after theorem head}
  {\thmname{#1} \thmnumber{#2}\thmnote{\normalfont#3}}% {theorem head spec}

\theoremstyle{cited}
%\newtheorem{citedthm}{Theorem}
\newtheorem{theorem}{Theorem}
\newtheorem{lemma}{Lemma}
\newtheorem{remark}{Remark}
\newtheorem{corollary}{Corollary}
\newtheorem{definition}{Definition}
\newtheorem{assumption}{Assumption}
\newtheorem{proposition}{Proposition}


\DeclareCaptionLabelSeparator{bperiod}{\textcolor[RGB]{70,126,166}{. }}
\captionsetup{format=hang, font=normalsize, labelsep=bperiod}
%\captionsetup{justification=centering}

\tolerance=2000
\emergencystretch=10pt
\frenchspacing

%% Indenting the first paragraph
\let\@afterindentfalse\@afterindenttrue
\@afterindenttrue

\makeatletter
\def\RemoveSpaces#1{\zap@space#1 \@empty}
\makeatother

\newcommand{\doi}[2]{
\def\temp{#2}\ifx\temp\empty
\hypersetup{urlcolor=black}
#1
\else
\hypersetup{urlcolor=blue}
\href{http://dx.doi.org/\RemoveSpaces{#2}}{#1}
\fi
}


\newcommand{\orcid}[1]{
\href{https://orcid.org/\RemoveSpaces{#1}}{\textcolor{white}{$\blacksquare$} \hskip -1.05pc 
\begin{minipage}{3.5mm}\includegraphics[width=3.5mm]{orcid_mark}\end{minipage}}
}		
\endinput
%%
%% End of file `IJCAS.cls'.
