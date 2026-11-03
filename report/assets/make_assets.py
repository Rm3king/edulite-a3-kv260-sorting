"""Draw repository presentation assets; measured data remain in metrics/."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from matplotlib.font_manager import FontProperties
import json
ROOT=Path(__file__).resolve().parent
FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
DARK='#102236';INK='#18354b';MUTED='#5c7182';TEAL='#007f80';ORANGE='#ba651f';BLUE='#3260ad';BG='#f4f7fb'
plt.rcParams['svg.fonttype']='path'
def canvas(w,h,face=BG):
 f,ax=plt.subplots(figsize=(w/100,h/100),dpi=100);f.patch.set_facecolor(face);ax.set_facecolor(face)
 f.subplots_adjust(0,0,1,1);ax.set_xlim(0,w);ax.set_ylim(h,0);ax.axis('off');return f,ax
def text(ax,x,y,s,size=22,c=INK,ha='left',weight='normal'):
 return ax.text(x,y,s,fontproperties=FONT,fontsize=size*.72,color=c,ha=ha,va='center',linespacing=1.7,fontweight=weight)
def box(ax,x,y,w,h,fill='white',edge='#d5e0ea',r=16):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle=f'round,pad=0,rounding_size={r}',linewidth=1.4,facecolor=fill,edgecolor=edge))
def arrow(ax,pts,c=MUTED,lw=2.2):
 for a,b in zip(pts[:-2],pts[1:-1]):ax.plot([a[0],b[0]],[a[1],b[1]],color=c,lw=lw)
 ax.add_patch(FancyArrowPatch(pts[-2],pts[-1],arrowstyle='-|>',mutation_scale=14,linewidth=lw,color=c))
def save(f,ax,name):
 f.canvas.draw();renderer=f.canvas.get_renderer();fw,fh=f.canvas.get_width_height()
 for t in ax.texts:
  bounds=t.get_window_extent(renderer)
  assert bounds.x0>=-2 and bounds.x1<=fw+2 and bounds.y0>=-2 and bounds.y1<=fh+2,(name,t.get_text(),bounds)
 f.savefig(ROOT/(name+'.png'),dpi=150,facecolor=f.get_facecolor())
 f.savefig(ROOT/(name+'.svg'),facecolor=f.get_facecolor())
 plt.close(f)

f,ax=canvas(1440,370,DARK)
for x,y,r in [(1280,85,120),(1280,85,170),(1280,85,225)]:ax.add_patch(Circle((x,y),r,fill=False,edgecolor='#223e54',lw=1.1))
ax.plot([48,48],[56,239],color='#28d1c3',lw=5)
text(ax,78,77,'AI PC  ×  FPGA  ×  ROBOTICS',20,'#8edbd4')
text(ax,78,145,'异构协同 · 具身智能',54,'white',weight='bold')
text(ax,80,212,'从中文指令到零件分拣与结果核验',26,'#d0deeb')
for x,label,color in [(80,'NPU · 零件分割','#12696e'),(365,'GPU · 本地千问','#2c4d7e'),(650,'CPU · 任务规划','#47536c'),(935,'FPGA · 周期执行','#8d542d')]:
 box(ax,x,274,250,55,color,color,10);text(ax,x+125,302,label,21,'white',ha='center')
save(f,ax,'project_banner')

f,ax=canvas(1440,850)
text(ax,45,48,'一次任务，五类计算单元协同完成',32,weight='bold')
text(ax,45,91,'AI PC 负责感知、理解与规划；KV260 负责板端视觉与周期执行。',22,MUTED)
for x,title,subtitle,col in [(40,'AI PC','Ryzen AI 9 HX PRO 370',TEAL),(510,'KV260','ARM + FPGA / PL',BLUE),(980,'实物工位','EDULite A3 · 双托盘',ORANGE)]:
 box(ax,x,145,420,570,'#eaf0f6','#d9e3ec',20);text(ax,x+25,180,title,28,col,weight='bold');text(ax,x+25,214,subtitle,19,MUTED)
box(ax,70,250,360,65,'#f9ffff','#a9d8d6');text(ax,250,282,'D435i · 彩色图像与深度',22,ha='center')
box(ax,70,345,360,80);text(ax,250,372,'NPU  /  YOLO11n-seg',24,TEAL,ha='center',weight='bold');text(ax,250,403,'四类零件实例分割',20,ha='center')
box(ax,70,465,360,90);text(ax,250,490,'CPU  /  定位与轨迹规划',24,TEAL,ha='center',weight='bold');text(ax,250,530,'三维位置 · IK · 几何校验',20,ha='center')
box(ax,70,605,360,80);text(ax,250,631,'GPU  /  本地千问',24,TEAL,ha='center',weight='bold');text(ax,250,663,'中文任务理解 · 执行后复核',20,ha='center')
arrow(ax,[(250,316),(250,344)],TEAL);arrow(ax,[(250,426),(250,464)],TEAL);arrow(ax,[(250,604),(250,556)],TEAL)
text(ax,263,580,'任务字段',15,MUTED)
box(ax,540,250,360,65,'#f9fcff','#b1c8e9');text(ax,720,282,'腕部 UVC · USB 直连',22,ha='center')
box(ax,540,355,360,115);text(ax,720,383,'ARM / PS',25,BLUE,ha='center',weight='bold');text(ax,720,420,'作业管理 · 零件定位',21,ha='center');text(ax,720,449,'偏差计算 · 抓取流程衔接',21,ha='center')
box(ax,540,535,360,150);text(ax,720,564,'FPGA / PL  ·  5 ms',25,BLUE,ha='center',weight='bold');text(ax,720,602,'轨迹求值 · CAN 调度 · 执行检查',19,ha='center');text(ax,720,634,'在线修正限幅 / 限速',19,ha='center');text(ax,720,663,'HLS 图像预处理（大螺丝）',19,ha='center')
arrow(ax,[(720,316),(720,354)],BLUE);arrow(ax,[(720,471),(720,534)],BLUE);text(ax,737,500,'AXI-Lite / DMA',16,MUTED)
arrow(ax,[(431,507),(485,507),(485,411),(539,411)],TEAL);text(ax,483,547,'作业包',15,TEAL,ha='center');text(ax,483,570,'以太网',15,TEAL,ha='center')
box(ax,1010,250,360,167,'#f9fbfd','#c5d4df');text(ax,1190,278,'头部观察整个双托盘工位',20,ha='center')
for x,label in [(1030,'A · 源盘'),(1200,'B · 目标盘')]:
 box(ax,x,308,150,86,'#d9eafb','#a9cce9',10);text(ax,x+75,333,label,17,ha='center')
 for xx in [x+39,x+76,x+112]:
  ax.plot([xx-8,xx+7],[367,356],color='#4a6275',lw=5,solid_capstyle='round')
text(ax,1190,452,'抓取 · 搬运 · 放置',22,ORANGE,ha='center')
box(ax,1010,535,360,150);text(ax,1190,567,'六轴机械臂 + 夹爪',25,ORANGE,ha='center',weight='bold');text(ax,1190,611,'六轴调整末端位置与朝向',20,ha='center');text(ax,1190,651,'第七电机完成夹持与释放',20,ha='center')
arrow(ax,[(1190,534),(1190,485)],ORANGE);arrow(ax,[(901,592),(1009,592)],BLUE);text(ax,954,564,'CAN',17,BLUE,ha='center');text(ax,954,619,'1 Mbps',15,BLUE,ha='center')
box(ax,40,743,1360,70,'#e3f2ee','#b7d8cf',14);text(ax,720,778,'结果核验：头部相机逐槽检查  →  本地千问生成复核说明  →  上位机显示执行结果',23,TEAL,ha='center')
save(f,ax,'system_architecture')

f,ax=canvas(1440,352)
text(ax,45,44,'从中文指令到结果核验',30,weight='bold')
text(ax,45,85,'任务描述、运动计算和实物反馈，各有明确的处理环节。',21,MUTED)
items=[('01','中文指令','规则解析 / 本地千问',TEAL),('02','识别定位','NPU 分割 / 深度标定',TEAL),('03','轨迹规划','CPU / 几何校验',TEAL),('04','腕部对准','ARM 视觉 / PL 修正',BLUE),('05','抓取放置','PL 周期执行 / CAN',BLUE),('06','结果核验','头部核对 / 千问复核',ORANGE)]
for i,(n,title,desc,col) in enumerate(items):
 x=40+i*230;box(ax,x,132,210,137,'white','#d7e2eb');text(ax,x+18,156,n,18,col);text(ax,x+105,198,title,26,col,ha='center',weight='bold');text(ax,x+105,242,desc,16,ha='center')
 if i<5:arrow(ax,[(x+211,199),(x+229,199)],MUTED,1.8)
text(ax,45,311,'上位机持续显示：识别数量、当前步骤、对准状态和每件零件的结果。',21,MUTED)
save(f,ax,'task_flow')
print('Created banner, architecture and task flow in PNG + SVG')
