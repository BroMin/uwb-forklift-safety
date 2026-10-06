#!/usr/bin/env python3
"""
uwb_web.py — 실시간 좌표 웹 뷰어

uwb_realtime.py 가 쓰는 CSV 를 뒤에서 따라 읽어 브라우저에 그린다.
실시간 스크립트를 건드리지 않으므로 검증된 버전을 그대로 쓸 수 있다.

사용법
    터미널 1:  python3 ~/uwb_realtime.py --k -0.784 --delta -0.304
    터미널 2:  python3 ~/uwb_web.py
    브라우저:  http://<라즈베리파이>:8000

    python3 ~/uwb_web.py --csv ~/uwb_logs/fix_xxx.csv    # 특정 파일
    python3 ~/uwb_web.py --port 9000 --true 1.0,0        # 참값 표시
"""

import argparse
import glob
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ANCHORS = {
    'RX1': (0.00, 0.00),
    'RX2': (0.00, -0.45),
    'RX3': (-0.50, -0.45),   
    'RX4': (-0.50, 0.00),    
}
MASTER = 'RX1'
LIMIT_M = 30.0                  # 이보다 먼 좌표는 표시하지 않음

STATE = {
    'pts': deque(maxlen=1200),      # (t, x, y, d1, rsl)
    'file': '',
    'rows': 0,
    'true': None,
    'bad': 0,
    'span': None,
    'lock': threading.Lock(),
}


def newest_csv():
    g = sorted(glob.glob(os.path.expanduser('~/uwb_logs/fix_*.csv')),
               key=os.path.getmtime)
    return g[-1] if g else None


def tail(path):
    """CSV 를 뒤에서 따라 읽는다. 파일이 새로 생기면 그쪽으로 옮겨간다."""
    cur = path
    f = None
    hdr = None
    while True:
        if f is None:
            if not cur or not os.path.exists(cur):
                time.sleep(0.5)
                cur = cur or newest_csv()
                continue
            f = open(cur, errors='ignore')
            hdr = f.readline().strip().split(',')
            with STATE['lock']:
                STATE['file'] = os.path.basename(cur)
                STATE['pts'].clear()
                STATE['rows'] = 0

        line = f.readline()
        if not line:
            # 더 최신 CSV 가 생겼으면 전환
            nw = newest_csv()
            if nw and nw != cur:
                f.close()
                f = None
                cur = nw
                continue
            time.sleep(0.15)
            continue

        p = line.strip().split(',')
        if len(p) < len(hdr):
            continue
        d = dict(zip(hdr, p))
        try:
            t = float(d['wall'])
            x = float(d['x'])
            y = float(d['y'])
            d1 = float(d.get('d1', 'nan'))
            rs = [float(v) for k, v in d.items() if k.startswith('rsl_')]
        except (ValueError, KeyError):
            continue
        # 좌표가 터무니없이 튄 행은 버린다. 남기면 자동 축이 망가진다.
        if not (abs(x) < LIMIT_M and abs(y) < LIMIT_M):
            with STATE['lock']:
                STATE['bad'] = STATE.get('bad', 0) + 1
            continue
        with STATE['lock']:
            STATE['pts'].append((t, x, y, d1, min(rs) if rs else 0.0))
            STATE['rows'] += 1


PAGE = """<!doctype html><meta charset=utf-8>
<title>UWB live</title>
<style>
body{margin:0;background:#14140f;color:#e8e6dd;font:14px system-ui,sans-serif}
#wrap{display:flex;flex-wrap:wrap;gap:14px;padding:14px}
canvas{background:#1c1c16;border:1px solid #3a3a30;border-radius:8px}
#info{min-width:230px;line-height:1.9}
b{font-weight:500;color:#fff}
.k{color:#8e8c80;display:inline-block;width:92px}
.warn{color:#f0997b}
</style>
<div id=wrap>
  <canvas id=map width=520 height=520></canvas>
  <div>
    <canvas id=ts width=520 height=200></canvas>
    <div id=info></div>
  </div>
</div>
<script>
const A=ANCHORS_JSON, TRUE=TRUE_JSON;
const map=document.getElementById('map'), mc=map.getContext('2d');
const ts=document.getElementById('ts'), tc=ts.getContext('2d');
const info=document.getElementById('info');
let pts=[], file='', rows=0, bad=0, SPAN=SPAN_JSON;

function bounds(){
  const ax=Object.values(A).map(p=>p[0]), ay=Object.values(A).map(p=>p[1]);
  const cx0=(Math.min(...ax)+Math.max(...ax))/2;
  const cy0=(Math.min(...ay)+Math.max(...ay))/2;
  let half=SPAN?SPAN/2:null;
  if(half===null){
    // 이상치에 흔들리지 않도록 중앙값 거리 기준으로 정한다
    const rec=pts.slice(-200);
    let r=1.5;
    if(rec.length){
      const ds=rec.map(p=>Math.hypot(p[1]-cx0,p[2]-cy0)).sort((a,b)=>a-b);
      r=ds[Math.floor(ds.length*0.8)]||1.5;
    }
    if(TRUE) r=Math.max(r,Math.hypot(TRUE[0]-cx0,TRUE[1]-cy0));
    half=Math.max(1.2,Math.min(8,r*1.4));
  }
  const x0=cx0-half,x1=cx0+half,y0=cy0-half,y1=cy0+half;
  const s=Math.min(map.width/(x1-x0),map.height/(y1-y0));
  return {x0,y0,s,cx:(map.width-(x1-x0)*s)/2,cy:(map.height-(y1-y0)*s)/2,half};
}
function drawMap(){
  const b=bounds();
  const X=v=>b.cx+(v-b.x0)*b.s, Y=v=>map.height-b.cy-(v-b.y0)*b.s;
  mc.clearRect(0,0,map.width,map.height);
  mc.strokeStyle='#2e2e26';mc.lineWidth=1;
  for(let g=-6;g<=6;g+=0.5){
    mc.beginPath();mc.moveTo(X(g),0);mc.lineTo(X(g),map.height);mc.stroke();
    mc.beginPath();mc.moveTo(0,Y(g));mc.lineTo(map.width,Y(g));mc.stroke();
  }
  const rec=pts.slice(-400);
  let off=0;
  rec.forEach((p,i)=>{
    const x=X(p[1]),y=Y(p[2]);
    if(x<0||x>map.width||y<0||y>map.height){off++;return;}
    mc.fillStyle='rgba(133,183,235,'+(0.12+0.5*i/rec.length)+')';
    mc.beginPath();mc.arc(x,y,2.5,0,7);mc.fill();
  });
  window.__off=off;
  if(TRUE){
    mc.strokeStyle='#E24B4A';mc.lineWidth=2;
    const x=X(TRUE[0]),y=Y(TRUE[1]);
    mc.beginPath();mc.moveTo(x-9,y);mc.lineTo(x+9,y);
    mc.moveTo(x,y-9);mc.lineTo(x,y+9);mc.stroke();
  }
  const cx0=(Math.min(...Object.values(A).map(p=>p[0]))+Math.max(...Object.values(A).map(p=>p[0])))/2;
  const cy0=(Math.min(...Object.values(A).map(p=>p[1]))+Math.max(...Object.values(A).map(p=>p[1])))/2;
  for(const [n,p] of Object.entries(A)){
    const x=X(p[0]),y=Y(p[1]);
    mc.fillStyle='#EF9F27';mc.fillRect(x-5,y-5,10,10);
    mc.fillStyle='#c9c7bd';mc.font='11px system-ui';
    mc.textAlign=p[0]<cx0?'right':'left';
    mc.fillText(n,x+(p[0]<cx0?-9:9),y+(p[1]<cy0?12:-6));
  }
  mc.textAlign='left';
  if(rec.length){
    const l=rec[rec.length-1];
    mc.strokeStyle='#5DCAA5';mc.lineWidth=2;
    mc.beginPath();mc.arc(X(l[1]),Y(l[2]),7,0,7);mc.stroke();
  }
}
function drawTs(){
  tc.clearRect(0,0,ts.width,ts.height);
  const rec=pts.slice(-400).filter(p=>isFinite(p[3]));
  if(rec.length<2)return;
  const so=rec.map(p=>p[3]).sort((a,b)=>a-b);
  let lo=so[Math.floor(so.length*0.02)],hi=so[Math.floor(so.length*0.98)];
  if(TRUE){const r=Math.hypot(TRUE[0],TRUE[1]);lo=Math.min(lo,r);hi=Math.max(hi,r);}
  const m=Math.max(0.05,(hi-lo)*0.15);lo-=m;hi+=m;
  const X=i=>20+i/(rec.length-1)*(ts.width-30);
  const Y=v=>ts.height-18-(v-lo)/(hi-lo)*(ts.height-32);
  tc.strokeStyle='#2e2e26';tc.lineWidth=1;
  tc.beginPath();tc.moveTo(20,Y(lo));tc.lineTo(ts.width-10,Y(lo));tc.stroke();
  if(TRUE){
    const r=Math.hypot(TRUE[0],TRUE[1]);
    tc.strokeStyle='#E24B4A';tc.setLineDash([4,4]);
    tc.beginPath();tc.moveTo(20,Y(r));tc.lineTo(ts.width-10,Y(r));tc.stroke();
    tc.setLineDash([]);
  }
  tc.strokeStyle='#85B7EB';tc.lineWidth=1.5;tc.beginPath();
  rec.forEach((p,i)=>i?tc.lineTo(X(i),Y(p[3])):tc.moveTo(X(i),Y(p[3])));
  tc.stroke();
  tc.fillStyle='#8e8c80';tc.font='11px system-ui';
  tc.fillText(hi.toFixed(2)+' m',22,14);
  tc.fillText(lo.toFixed(2)+' m',22,ts.height-4);
  tc.fillText('TWR 거리',ts.width-70,14);
}
function med(a){const b=[...a].sort((x,y)=>x-y);return b[b.length>>1];}
function stats(){
  const rec=pts.slice(-400);
  if(!rec.length){info.innerHTML='<span class=k>대기</span> CSV 수신 없음';return;}
  const xs=rec.map(p=>p[1]),ys=rec.map(p=>p[2]),ds=rec.map(p=>p[3]);
  const mx=med(xs),my=med(ys);
  const sp=rec.map(p=>Math.hypot(p[1]-mx,p[2]-my)).sort((a,b)=>a-b);
  const s95=sp[Math.floor(sp.length*0.95)]||0;
  const s50=sp[Math.floor(sp.length*0.50)]||0;
  const l=rec[rec.length-1];
  let h='<span class=k>파일</span>'+file+'<br>'
   +'<span class=k>좌표 수</span>'+rows.toLocaleString()+'<br>'
   +'<span class=k>현재</span><b>('+l[1].toFixed(3)+', '+l[2].toFixed(3)+')</b> m<br>'
   +'<span class=k>중앙값</span><b>('+mx.toFixed(3)+', '+my.toFixed(3)+')</b> m<br>'
   +'<span class=k>TWR 거리</span>'+med(ds).toFixed(3)+' m<br>'
   +'<span class=k>산포 중앙</span>'+(s50*100).toFixed(1)+' cm<br>'
   +'<span class=k>산포 P95</span>'+(s95*100).toFixed(1)+' cm<br>'
   +'<span class=k>화면 밖</span>'+(window.__off||0)+' / 제외 '+bad+'<br>'
   +'<span class=k>rsl</span>'+l[4].toFixed(1)+' dBm'
   +(l[4]<-70?' <span class=warn>약함</span>':'')+'<br>';
  if(TRUE){
    const e=Math.hypot(mx-TRUE[0],my-TRUE[1]);
    h+='<span class=k>참값</span>('+TRUE[0]+', '+TRUE[1]+') m<br>'
      +'<span class=k>편향</span><b>'+(e*100).toFixed(1)+' cm</b>';
  }
  info.innerHTML=h;
}
async function tick(){
  try{
    const r=await fetch('/data');const j=await r.json();
    pts=j.pts;file=j.file;rows=j.rows;bad=j.bad||0;
    if(j.span)SPAN=j.span;
    drawMap();drawTs();stats();
  }catch(e){}
  setTimeout(tick,250);
}
tick();
</script>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/data':
            with STATE['lock']:
                body = json.dumps({
                    'pts': list(STATE['pts'])[-400:],
                    'file': STATE['file'],
                    'rows': STATE['rows'],
                    'bad': STATE.get('bad', 0),
                    'span': STATE['span'],
                }).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        page = (PAGE
                .replace('ANCHORS_JSON', json.dumps(ANCHORS))
                .replace('TRUE_JSON', json.dumps(STATE['true']))
                .replace('SPAN_JSON', json.dumps(STATE['span'])))
        body = page.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', help='기본값: ~/uwb_logs 의 최신 fix_*.csv')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--true', help='참값 "x,y" — 표시용')
    ap.add_argument('--span', type=float, default=None,
                    help='화면 한 변의 길이 (m). 미지정 시 자동')
    args = ap.parse_args()

    STATE['span'] = args.span
    if args.true:
        STATE['true'] = [float(v) for v in args.true.split(',')]

    path = os.path.expanduser(args.csv) if args.csv else newest_csv()
    print(f'CSV  {path or "(대기 — uwb_realtime.py 를 먼저 실행하세요)"}')
    threading.Thread(target=tail, args=(path,), daemon=True).start()

    print(f'브라우저에서 열기')
    print(f'  http://<라즈베리파이 주소>:{args.port}')
    print(f'  예) http://192.168.50.43:{args.port}')
    print('Ctrl+C 로 종료')
    try:
        ThreadingHTTPServer(('0.0.0.0', args.port), H).serve_forever()
    except KeyboardInterrupt:
        print('\n종료')


if __name__ == '__main__':
    main()