# 西北工业大学 - 学士学位论文LaTeX模板

## 1. 简介

本项目是西北工业大学-学士学位论文的LaTex模版，这个模版有如下特点：
* 使用XeLaTeX，方便字体设置。中文支持使用xeCJK包。
* 用pgf/tikz宏包画成矢量校徽。
* 自制nwpulogo矢量字体，即毛体的“西北工业大学”六个字。
* 已做好封面



## 2. 安装TexLive

**对于命令行操作不熟悉的同学请自行SOS**

先安装texlive，可以在tuna网站下载（https://mirrors.tuna.tsinghua.edu.cn/CTAN/systems/texlive/Images/）。

安装说明：https://zhuanlan.zhihu.com/p/64555335



如果使用Linux操作系统，可以执行如下命令安装：

```
sudo apt install texlive-full texlive-xetex
sudo apt install texstudio
```



## 3. 编译

进入当前目录，进行编译：

```
  make all
```
即可生成`main.pdf`



## 4. 注意事项

* 如果系统中没有make命令，那么按顺序执行如下命令：
```
    xelatex main.tex
    bibtex main.tex
    xelatex main.tex
    xelatex main.tex
```
* Linux和Windows下都推荐安装Texlive 2020。

* 将`nwpulogo.ttf`放到`fc-list`可以找到的地方。Linux下放到`~/.fonts`，Windows下放到`C:\windows\Fonts`。Linux下安装完成后执行：

```
mkdir ~/.fonts
cp nwpulogo.ttf ~/.fonts
cd ~/.fonts
fc-cache -fv
```



## 5. 字体问题

需要将以下列出的字体装全（怎么装？这可得靠自己机灵了。其实都可以用系统中的字体替换掉），可以用`fc-list`全部认出，才能通过编译。

尽量避免使用版权字体，大部分采用了开源的字体。

其中不会有版权问题的字体有：



## 6. 参考资料
Latex的用法说明等材料： https://gitee.com/pi-lab/resources/tree/master/books/latex

更多内容请移步 
* http://code.google.com/p/nwputhesis/
* https://github.com/GFrankenstein/NJU-Thesis-LaTeX-Template
