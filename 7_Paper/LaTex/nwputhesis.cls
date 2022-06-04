\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{nwputhesis}[2018/04/30 A template for nwpu bachelor thesis v1.0 made by Wang Aixue]
\LoadClass[oneside, a4paper, zihao=-4, UTF8, openright]{ctexbook}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%协议：MIT协议（http://opensource.org/licenses/MIT）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%对于包的使用


%超链接
\RequirePackage[colorlinks,linkcolor=black, citecolor=black, urlcolor=black]{hyperref}
%板式
\RequirePackage[top=0.8in,
	    bottom=1.6in,
	    left=1.2in,
	    right=1.2in]{geometry}
%图片
\RequirePackage{graphicx}
%图表标题
\RequirePackage{bicaption}
%子图
\RequirePackage{subcaption}
%页眉页脚控制
\RequirePackage{fancyhdr}
%非汉字字体设置
\RequirePackage{fontspec}
%控制参考文献格式
\RequirePackage[square,super, comma, sort&compress, numbers]{natbib}
\RequirePackage{hypernat}
%计数器控制
\RequirePackage{amsmath}
\RequirePackage{mathtools}
%表格控制
\RequirePackage{multirow} 
\RequirePackage{array}
\RequirePackage{booktabs}
% use xifthen
\RequirePackage{xifthen}
\RequirePackage{tikz}
%插入代码
\RequirePackage{listings}
%设置列表格式
\RequirePackage{enumitem}
%插入代码
\RequirePackage{listings}
\RequirePackage{diagbox}
\RequirePackage{bigstrut}
% 控制目录的宏包
\RequirePackage{titletoc}
\RequirePackage{titlesec}
%
\RequirePackage{setspace}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%论文行间距为1.5倍（可能需要用\selectfont）

\linespread{1.25}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 重定义字号命令
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\newcommand{\chuhao}{\fontsize{42pt}{63pt}\selectfont}    % 初号, 1.5倍行距
\newcommand{\yihao}{\fontsize{26pt}{36pt}\selectfont}    % 一号, 1.4倍行距
\newcommand{\erhao}{\fontsize{22pt}{28pt}\selectfont}    % 二号, 1.25倍行距
\newcommand{\xiaoer}{\fontsize{18pt}{18pt}\selectfont}    % 小二, 单倍行距
\newcommand{\sanhao}{\fontsize{16pt}{24pt}\selectfont}    % 三号, 1.5倍行距
\newcommand{\xiaosan}{\fontsize{15pt}{22pt}\selectfont}    % 小三, 1.5倍行距
\newcommand{\sihao}{\fontsize{14pt}{21pt}\selectfont}    % 四号, 1.5倍行距
\newcommand{\banxiaosi}{\fontsize{13pt}{16.25pt}\selectfont}    % 半小四, 1.25倍行距
\newcommand{\xiaosi}{\fontsize{12.5pt}{12.5pt}\selectfont}    % 小四, 1.2倍行距
\newcommand{\dawuhao}{\fontsize{11pt}{11pt}\selectfont}    % 大五号, 单倍行距
\newcommand{\wuhao}{\fontsize{10.5pt}{10.5pt}\selectfont}    % 五号, 单倍行距
\newcommand{\xiaowu}{\fontsize{9pt}{9pt}\selectfont}		% 小五号
\newcommand{\tabincell}[2]{\begin{tabular}{@{}#1@{}}#2\end{tabular}}
\setCJKfamilyfont{nwpu}{nwpulogo} 
%%页眉页脚开启
\fancypagestyle{plain}{
	\fancyhf{}
	\fancyhead[C]{\sanhao\raisebox{0.04cm}{\CJKfamily{nwpu} 西北工业大学} \CJKfamily{zhsong} 本科毕业设计论文}
	 \fancyfoot[C]{\wuhao \thepage}
}


%论文中出现的所有数字和字母如无特殊要求都用Times New Roman体。
%设置正文非汉字的字体为Times New Roman
\setmainfont{Times New Roman}
%设置正文非汉字的粗体字体为Times New Roman
\setsansfont{Times New Roman}

%设置英文图为Fig，表格为Tab
\DeclareCaptionOption{english}[]{
\renewcommand\figurename{Fig.}
\renewcommand\tablename{Tab}}
\captionsetup[bi-second]{english}

%一定要在这里声明space，直接声明不好使，小五号
\captionsetup{font={footnotesize,bf},labelsep=space}


%字体设置
%章标题 黑体三号
%节标题 黑体四号
%小节标题 黑体小四
\ctexset{chapter={
	format={\centering\sanhao\CJKfamily{zhhei}},
	nameformat={\sanhao\CJKfamily{zhhei}},
	titleformat={\sanhao\CJKfamily{zhhei}},
	beforeskip={15\p@},
	afterskip={12\p@},
	}
}
\ctexset{section={
	format={\sihao\CJKfamily{zhhei}},
	afterskip={1.0ex \@plus .2ex},
	}
}
\ctexset{subsection={
	format={\xiaosi\CJKfamily{zhhei}},
	indent={0\ccwd},
	afterskip={1.0ex \@plus .2ex},
	}
}
\ctexset{subsubsection={
	format={\wuhao\CJKfamily{zhhei}},
	indent={0\ccwd},
	afterskip={1.0ex \@plus .2ex},
	}
}
%设置图片编号随着章节自动清零
\numberwithin{figure}{chapter}
%设置公式编号随着章节自动清零
\numberwithin{equation}{chapter}
%设置表编号随着章节自动清零
\numberwithin{table}{chapter}
%定义图的编号为章节号码-序号
\renewcommand{\thefigure}{\thechapter-\arabic{figure}}
%定义公式的编号为章节号码-序号
\renewcommand{\theequation}{\thechapter-\arabic{equation}}
%定义表的编号为章节号码-序号
\renewcommand{\thetable}{\thechapter-\arabic{table}}

%改变figure为图
\renewcommand{\figureautorefname}{图}

%改变equation为式
\renewcommand{\equationautorefname}{式}
%\renewcommand{\subsectionautorefname}{节}
%\renewcommand{\subsectionautorefname}{章}



%\bibliographystyle{GBT7714-2005NLang-UTF8}
%\bibliographystyle{nwputhesis}
%\bibliographystyle{gbt7714-author-year}
\bibliographystyle{gbt7714-numerical}

%代码报lsting格式
%\lstset{numbers=none,
%  numberstyle=\scriptsize,
%  frame=lines,
%  flexiblecolumns=false,
%  language=Python,
%  basicstyle=\ttfamily\small,
%  breaklines=true,
%  extendedchars=true,
%  escapechar=\%,
%  texcl=true,
%  showstringspaces=true,
%  keywordstyle=\bfseries,
%  tabsize=4}

% 汉语作者
\newcommand{\nwpuAuthor}[1]{\def\nwpuAuthorString{#1}}
% 专业
\newcommand{\nwpuMajor}[1]{\def\nwpuMajorString{#1}}
%教师
\newcommand{\nwpuSupervisorFirst}[1]{\def\nwpuSupervisorFirstString{#1}}
% 毕业时间
\newcommand{\nwpuGraduateTime}[1]{\def\nwpuGraduateTimeString{#1}}
% 标题
\newcommand{\nwpuTitleFirstLine}[1]{\def\nwpuTitleFirstLineString{#1}}

\newcolumntype{C}[1]{>{\centering\let\newline\\\arraybackslash\hspace{0pt}}m{#1}}

%%%%%%%%%%%%%%%%%%%%%%%%%
%%目录设置
%%%%%%%%%%%%%%%%%%%%%%%%%%
\setcounter{tocdepth}{4}
\setcounter{secnumdepth}{4}  %% 章节编号深度 (part 对应 -1)
%
\titlecontents{chapter}
				[0em]
				{\vspace*{3mm}\bf \xiaosi}
				{\makebox[4em][l]{\contentslabel{0em}}}
				{}
				{\titlerule*[0.5pc]{$\cdot$}\contentspage}

\titlecontents{section}  
              [4.1em]  
              {\xiaosi \CJKfamily{zhsong}}% 标题格式 
              {\contentslabel{2em}}%  标题标志
              {}%  无序号标题
              {\titlerule*[0.5pc]{$\cdot$}\contentspage}%  指引线与页码
\titlecontents{subsection}  
              [6em]  
              {\xiaosi \CJKfamily{zhsong}}% 标题格式 
              {\contentslabel{3em}}%  标题标志
              {}%  无序号标题
              {\titlerule*[0.5pc]{$\cdot$}\contentspage}%  指引线与页码
\titlecontents{subsubsection}  
              [8em]  
              {\xiaosi \CJKfamily{zhsong}}% 标题格式 
              {\contentslabel{4em}}%  标题标志
              {}%  无序号标题
              {\titlerule*[0.5pc]{$\cdot$}\contentspage}%  指引线与页码
              

%%%%%%%%%%%%%%%%%%%%%%%%%
%% References
%%%%%%%%%%%%%%%%%%%%%%%%%         
\setlength{\bibsep}{0.5ex}  %vertical spacing between references

