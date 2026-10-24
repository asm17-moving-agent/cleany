from pathlib import Path
import json
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;O=R/'media';O.mkdir(exist_ok=True);S=R/'weight-sweep';j=json.loads((S/'00005-2.input.json').read_text());nx,ny=j['nx'],j['ny'];a=Image.frombytes('L',(nx,ny),(S/'00005.bin').read_bytes()).transpose(Image.Transpose.FLIP_TOP_BOTTOM);im=a.point(lambda c:45 if c>=254 else (130 if c==253 else 249-round(c*.3))).convert('RGB');ox,oy,_=j['origin'];res=j['resolution'];font=ImageFont.truetype('/usr/share/fonts/google-noto/NotoSans-Regular.ttf',17);canvas=Image.new('RGB',(1400,650),'white')
paths={w:json.loads((S/f'00005-{w}.output.json').read_text())['poses'] for w in [2,5]};colors={2:(213,111,27),5:(35,101,210)}
for col,limits in enumerate([(-18,0,-8,4),(-7.5,-4,-7.5,-4)]):
 x0,x1,y0,y1=limits;scale=min(660/(x1-x0),520/(y1-y0));pw,ph=round((x1-x0)*scale),round((y1-y0)*scale);left=col*700+(700-pw)//2;top=100+(540-ph)//2
 crop=im.crop((round((x0-ox)/res),round(ny-(y1-oy)/res),round((x1-ox)/res),round(ny-(y0-oy)/res))).resize((pw,ph),Image.Resampling.NEAREST);d=ImageDraw.Draw(crop)
 def tr(x,y):return (x-x0)*scale,(y1-y)*scale
 for w,p in paths.items():d.line([tr(*v[:2]) for v in p],fill=colors[w],width=3)
 canvas.paste(crop,(left,top));draw=ImageDraw.Draw(canvas);draw.text((col*700+20,15),'Same frozen global costmap | actual native Theta* plans',font=font,fill=(35,35,35));draw.text((col*700+20,42),'Orange: traversal weight 2 | blue: weight 5',font=font,fill=(35,35,35));draw.text((col*700+20,69),'Full route' if col==0 else 'Failure corner, enlarged',font=font,fill=(35,35,35))
canvas.save(O/'theta-static-clearance-before-after.png')
