// Pure screen-space interaction math. Display samples are never exported as XODR.
export function toScreen(p, v) { return [v.x+p[0]*v.k, v.y-p[1]*v.k]; }
export function toWorld(p, v) { return [(p[0]-v.x)/v.k, -(p[1]-v.y)/v.k]; }
export function dragValue(startValue, start, now, normal, scale) {
  return startValue+((now[0]-start[0])*normal[0]-(now[1]-start[1])*normal[1])/scale;
}
export function zoomAt(v, point, factor) {
  const k=Math.max(.15,Math.min(150,v.k*factor));
  return {k,x:point[0]-(point[0]-v.x)*k/v.k,y:point[1]-(point[1]-v.y)*k/v.k};
}
export function resizeView(v, oldSize, size) {
  return {...v,x:v.x+(size[0]-oldSize[0])/2,y:v.y+(size[1]-oldSize[1])/2};
}
export function segmentDistance(p,a,b) {
  const dx=b[0]-a[0],dy=b[1]-a[1],d=dx*dx+dy*dy;
  const t=d?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d)):0;
  return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
}
export function pickLines(point, geometry, view, radius=10) {
  return Object.entries(geometry||{}).map(([id,g])=>{
    let distance=Infinity;
    for(const line of g.lines)for(let i=1;i<line.length;i++)
      distance=Math.min(distance,segmentDistance(point,toScreen(line[i-1],view),toScreen(line[i],view)));
    return {id,distance};
  }).filter(h=>h.distance<=radius).sort((a,b)=>a.distance-b.distance||a.id.localeCompare(b.id));
}
// One request in flight: backend caches only its latest preview token.
// Superseded/cancelled replies must not be rendered or applied.
export class LatestPreview {
  constructor(request, result, error, change=()=>{}) {
    Object.assign(this,{request,result,error,change,serial:0,running:false,waiting:null});
  }
  get pending(){return this.running||!!this.waiting}
  submit(payload){this.waiting={payload,serial:++this.serial};this.change();void this.pump();}
  cancel(){this.serial++;this.waiting=null;this.change();}
  async pump(){
    if(this.running)return;
    this.running=true;this.change();
    try {
      while(this.waiting){
        const job=this.waiting;this.waiting=null;
        try{const value=await this.request(job.payload);
          if(job.serial===this.serial)this.result(value,job.payload);
        }catch(e){if(job.serial===this.serial)this.error(e);}
      }
    }finally{this.running=false;this.change();}
  }
}

async function mountWorkbench(){
  const $=id=>document.getElementById(id), NS='http://www.w3.org/2000/svg';
  const fragment=new URLSearchParams(location.hash.slice(1));
  const token=fragment.get('token')||sessionStorage.getItem('mapforge-token')||'';
  if(token)sessionStorage.setItem('mapforge-token',token);
  history.replaceState(null,'',location.pathname);
  const svg=$('map'),world=$('world'),controls=$('controls');
  let project,state,selected=null,preview=null,exportResult=null,busy=false;
  let view={x:0,y:0,k:1},size=[0,0],cameraMode='full',gesture=null,hover=null;
  let mode='overlay',sourcePeek=false,frame=null,previewTimer=null,intent=null;
  const queue=new LatestPreview(
    payload=>api('/api/action/preview',payload),
    (p,payload)=>{
      if(payload.revision!==state.revision||payload.handle!==selected?.id)return;
      preview=p;$('delta').value=p.value.toFixed(3);
      $('previewInfo').textContent='位移 '+p.value.toFixed(3)+' m；修改 '+p.guard.changed_width_records+
        ' 条已有 width。紫色为实际候选 XML 读回，尚未应用。';
      status('预览就绪。确认紫色线后按 Enter 应用，Esc 取消。');render();updateButtons();
    },
    e=>{preview=null;status('预览被拒绝：'+e.message,true);$('previewInfo').textContent=e.message;render();},
    ()=>{if(state)updateButtons();}
  );
  async function api(path,body){
    const headers={Authorization:'Bearer '+token};
    if(body){headers['Content-Type']='application/json';headers['X-Mapforge-CSRF']=token;}
    const r=await fetch(path,{method:body?'POST':'GET',headers,body:body?JSON.stringify(body):undefined});
    const data=await r.json();if(!r.ok)throw new Error(data.detail||'HTTP '+r.status);return data;
  }
  const action=(name,extra={})=>api('/api/action/'+name,{revision:state.revision,...extra});
  function status(text,error=false){$('status').textContent=text;$('status').classList.toggle('error',error);}
  function pending(){return queue.pending||previewTimer!==null;}
  function hasIntent(){return intent!==null||preview!==null;}
  function updateButtons(){
    const locked=busy||!state||pending();
    for(const id of ['save','export','reopen'])$(id).disabled=locked||hasIntent();
    $('undo').disabled=locked||!state?.can_undo;$('redo').disabled=locked||!state?.can_redo;
    for(const id of ['apply','applyDock'])$(id).disabled=locked||!preview||!!gesture;
    for(const id of ['discard','cancelDock'])$(id).disabled=busy||!hasIntent();
    for(const id of ['preview','delta','minusCm','plusCm'])$(id).disabled=busy||!selected;
    $('road').disabled=busy;$('download').disabled=busy;$('focusSelected').disabled=!selected;
    $('liveValue').textContent=hasIntent()?
      (pending()?'计算中 · ':preview?'待应用 · ':'未应用意图 · ')+(intent??preview?.value??0).toFixed(3)+' m':
      selected?'已应用位移 '+(state.values[selected.id]||0).toFixed(3)+' m':'青色可选 · 其他对象锁定';
    $('previewDock').dataset.state=pending()?'calculating':preview?'preview':'idle';
  }
  async function work(fn){
    if(busy||pending())return;busy=true;updateButtons();
    try{await fn()}catch(e){status(e.message,true)}finally{busy=false;updateButtons();}
  }
  function clearIntent(){
    clearTimeout(previewTimer);previewTimer=null;queue.cancel();preview=null;intent=null;gesture=null;
    if(selected)$('delta').value=(state.values[selected.id]||0).toFixed(3);
    $('previewInfo').textContent='拖动双箭头柄或输入位移后预览；不会自动应用。';
  }
  function cancel(){
    clearIntent();status('预览已取消，当前已应用版本未改变。');render();updateButtons();
  }
  function requestPreview(value,immediate=false){
    if(!selected||busy)return;
    clearTimeout(previewTimer);previewTimer=null;queue.cancel();preview=null;intent=value;
    $('delta').value=Number.isFinite(value)?value.toFixed(3):'';
    if(!Number.isFinite(value)){status('请输入有效位移。',true);render();updateButtons();return;}
    const payload={revision:state.revision,handle:selected.id,value};
    const send=()=>{previewTimer=null;queue.submit(payload);};
    if(immediate)send();else previewTimer=setTimeout(send,160);
    status('正在计算实际 XML 预览；未应用。');render();updateButtons();
  }
  function element(tag,attrs,parent=world,text){
    const e=document.createElementNS(NS,tag);
    for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);
    if(text!==undefined)e.textContent=text;parent.append(e);return e;
  }
  function line(points,color,width=1,opacity=1,dash){
    if(points.length<2)return;
    element('polyline',{points:points.map(p=>p[0]+','+-p[1]).join(' '),stroke:color,
      'stroke-width':width,opacity,class:'edge',...(dash?{'stroke-dasharray':dash}:{})});
  }
  function roadsDraw(roads,color,width,alpha,centers=false){
    for(const r of roads){
      if(r.junction&&!$('connectors').checked)continue;
      for(const p of centers?r.centers:r.edges)line(p,color,width,alpha,centers?'4 6':null);
    }
  }
  function handleXY(h,value=state.values[h.id]||0){return [h.xy[0]+value*h.normal[0],h.xy[1]+value*h.normal[1]];}
  function visibleGeometry(){return preview?.selection||state.selection||{};}
  function sourceOnly(){return sourcePeek||mode==='source';}
  function render(){if(frame!==null)return;frame=requestAnimationFrame(()=>{frame=null;draw();});}
  function draw(){
    if(!state)return;world.replaceChildren();controls.replaceChildren();
    world.setAttribute('transform','translate('+view.x+' '+view.y+') scale('+view.k+')');
    const source=sourceOnly(),showSource=source||mode==='overlay';
    if(!source&&$('baseline').checked)roadsDraw(project.baseline,'#97a4ae',1,.45);
    if(showSource&&$('sourceEdges').checked)for(const p of project.source.boundaries)line(p.points,'#d68b32',1.5,.9);
    if(showSource&&$('sourcePaths').checked)for(const p of project.source.paths)line(p.points,'#269978',1,.65,'3 5');
    const displayed=preview?state.roads.map(r=>r.id===preview.road.id?preview.road:r):state.roads;
    if(!source&&$('candidate').checked)roadsDraw(displayed,'#507caa',1.05,.72);
    if(!source&&$('centers').checked)roadsDraw(displayed,'#7d9fbf',.9,.55,true);
    if(!source&&preview)for(const p of state.selection?.[selected.id]?.lines||[])
      line(p,'#8696a4',1.1,.8,'4 4');
    if(!source){
      for(const [id,g] of Object.entries(visibleGeometry())){
        const active=id===selected?.id,hot=id===hover;
        for(const p of g.lines)line(p,active?(preview?'#b32d94':'#087c91'):'#22a5ad',active?3:hot?3:1.7,active?1:.8,active?null:'5 4');
      }
      // Older server sessions remain usable, but cannot invent line ownership.
      if(!state.selection)for(const h of state.handles){
        const p=toScreen(handleXY(h),view);
        element('circle',{cx:p[0],cy:p[1],r:8,fill:'#1c9da8','data-handle':h.id},controls);
      }
      if(selected){
        const v=gesture?.kind==='handle'?gesture.value:(intent??state.values[selected.id]??0);
        const p=toScreen(handleXY(selected,v),view);
        const g=element('g',{'data-handle':selected.id,class:'control-handle',
          'aria-label':'拖动共享分界线 '+selected.id,transform:'translate('+p[0]+' '+p[1]+')'},controls);
        element('circle',{r:23,fill:'#fff',opacity:.9},g);
        const angle=Math.atan2(-selected.normal[1],selected.normal[0])*180/Math.PI;
        const arrow=element('g',{transform:'rotate('+angle+')'},g);
        element('path',{d:'M -19 0 L 19 0 M -12 -5 L -19 0 L -12 5 M 12 -5 L 19 0 L 12 5',
          stroke:hasIntent()?'#b32d94':'#087c91','stroke-width':2.5,fill:'none'},arrow);
        element('circle',{r:6,fill:hasIntent()?'#b32d94':'#087c91',stroke:'#fff','stroke-width':2},g);
        element('circle',{r:23,fill:'transparent'},g);
        element('text',{x:p[0]+29,y:p[1]-14,class:'control-label'},controls,
          selected.id+(view.k<4?' · 点柄放大':' · 横向拖动'));
        if(gesture?.kind==='handle')element('text',{x:p[0]+29,y:p[1]+5,class:'control-label'},controls,v.toFixed(3)+' m');
        for(const end of visibleGeometry()[selected.id]?.ends||[]){
          const q=toScreen(end,view);element('text',{x:q[0]-6,y:q[1]+4,class:'end-lock'},controls,'▣');
        }
      }
    }
    $('zoomLabel').textContent=view.k.toFixed(1)+' px/m';
    $('scale').textContent=(70/view.k).toFixed(1)+' m';
  }
  function updateHandleList(){
    $('handleList').replaceChildren();
    const hs=state.handles.filter(h=>h.road===$('road').value);
    for(const h of hs){
      const b=document.createElement('button');b.className='handle-item';b.dataset.handleId=h.id;
      b.textContent='分界 '+h.lane+' ↔ '+h.neighbor;
      b.setAttribute('aria-pressed',String(h.id===selected?.id));
      b.onclick=()=>{choose(h);focusSelected();closePanels();svg.focus();};$('handleList').append(b);
    }
    if(!hs.length)$('handleList').textContent='本路未开放可编辑分界线。';
  }
  function choose(h){
    clearIntent();selected=h||null;hover=null;$('pickMenu').hidden=true;
    if(h)$('road').value=h.road;
    $('selected').textContent=h?'道路 '+h.road+' / 分界 '+h.lane+' ↔ '+h.neighbor:'尚未选择';
    $('selectionTitle').textContent=h?'已选：道路 '+h.road+' · 分界 '+h.lane+' ↔ '+h.neighbor:'点青色分界线选中';
    $('selectionHint').textContent=h?'拖双箭头柄 · Enter 应用 · Esc 取消':'选线 → 放大 → 拖双箭头柄';
    $('handleInfo').textContent=h?'控制站位 s='+h.s.toFixed(1)+'m；锁定区间两端。\n作用范围 '+h.knots[0].toFixed(1)+'–'+h.knots[4].toFixed(1)+'m\n4 个长三次增量区间，最短 '+h.min_basis_span_m.toFixed(1)+'m。':'只有青色共享分界线可编辑。';
    $('delta').value=h?(state.values[h.id]||0).toFixed(3):'0';
    updateHandleList();render();updateButtons();
  }
  function refresh(next){
    const key=selected?.id;clearIntent();state=next;exportResult=null;
    $('download').hidden=true;$('exportResult').textContent='';
    $('version').textContent='版本 '+state.revision.slice(0,8)+' · '+(state.saved_revision===state.revision?'已保存':'未保存')+
      '\nXML '+state.sha256.slice(0,12)+'…';
    $('metrics').textContent='参考线原语 '+state.complexity.geometry+'（最短 '+state.complexity.min_geometry_m.toFixed(2)+
      'm）\nwidth '+state.complexity.width+' · offset '+state.complexity.laneOffset+
      '\n本次增量不增加参考线 / width 记录。旧图密集横断面仍存在。';
    choose(state.handles.find(h=>h.id===key));
  }
  function fit(roadId){
    if(!state)return;cameraMode=roadId?'road':'full';
    const points=(roadId?state.roads.filter(r=>r.id===roadId):state.roads).flatMap(r=>r.edges.flat());
    if(!points.length)return;
    let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
    for(const p of points){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1]);}
    const b=svg.getBoundingClientRect();
    view.k=Math.max(.15,Math.min((b.width-90)/Math.max(xmax-xmin,20),(b.height-285)/Math.max(ymax-ymin,20)));
    view.x=b.width/2-(xmin+xmax)/2*view.k;view.y=(b.height+15)/2+(ymin+ymax)/2*view.k;
    render();
  }
  function focusSelected(){
    if(!selected)return;cameraMode='detail';
    const box=svg.getBoundingClientRect(),p=handleXY(selected,intent??state.values[selected.id]??0);
    view.k=Math.max(4,Math.min(12,(box.width-100)/40,(box.height-260)/35));
    view.x=box.width/2-p[0]*view.k;view.y=box.height/2+p[1]*view.k;
    status('拖动双箭头控制柄调整整段分界线；相邻车道宽度联动。');render();
  }
  function localPoint(e){const b=svg.getBoundingClientRect();return [e.clientX-b.left,e.clientY-b.top];}
  function closePanels(){
    for(const id of ['leftPanel','rightPanel'])$(id).classList.remove('open');
    $('panelShade').hidden=true;$('toggleLeft').setAttribute('aria-expanded','false');$('toggleRight').setAttribute('aria-expanded','false');
  }
  function togglePanel(which){
    const open=$(which).classList.contains('open');closePanels();
    if(!open){$(which).classList.add('open');$('panelShade').hidden=false;
      $(which==='leftPanel'?'toggleLeft':'toggleRight').setAttribute('aria-expanded','true');}
  }
  function setMode(next){
    mode=next;sourcePeek=false;
    for(const [id,value] of [['modeOverlay','overlay'],['modeSource','source'],['modeResult','result']])
      $(id).setAttribute('aria-pressed',String(mode===value));
    if(next==='source')status('源图只读；切回叠图或结果再编辑。');
    render();
  }
  function picked(point){
    const hits=pickLines(point,visibleGeometry(),view);
    if(!hits.length){status('这里未开放编辑。请点青色分界线；外缘与路口仍锁定。');return;}
    const close=hits.filter(h=>h.distance<=hits[0].distance+3);
    if(close.length===1){choose(state.handles.find(h=>h.id===close[0].id));status('已选中分界线。点击“放大编辑”，再拖动双箭头柄。');return;}
    const menu=$('pickMenu');menu.replaceChildren();
    const title=document.createElement('small');title.textContent='线较密，请确认要编辑哪一条';menu.append(title);
    for(const hit of close){
      const h=state.handles.find(h=>h.id===hit.id),b=document.createElement('button');
      b.textContent='道路 '+h.road+' · '+h.lane+' ↔ '+h.neighbor;
      b.onclick=()=>{choose(h);focusSelected();svg.focus();};menu.append(b);
    }
    menu.style.left=Math.max(10,Math.min(point[0],svg.clientWidth-240))+'px';
    menu.style.top=Math.max(120,Math.min(point[1],svg.clientHeight-240))+'px';menu.hidden=false;
  }
  svg.addEventListener('pointerdown',e=>{
    if(e.button!==0||busy||!state)return;svg.focus();$('pickMenu').hidden=true;
    const p=localPoint(e),handle=e.target.closest('[data-handle]');
    if(handle&&!sourceOnly()){
      const h=state.handles.find(h=>h.id===handle.dataset.handle);if(h?.id!==selected?.id)choose(h);
      if(view.k<4){focusSelected();gesture={kind:'focus'};return;}
      const startValue=intent??state.values[h.id]??0;
      gesture={kind:'handle',start:p,startValue,value:startValue,moved:false};
    }else gesture={kind:'pan',start:p,view:{...view},moved:false};
    svg.setPointerCapture(e.pointerId);
  });
  svg.addEventListener('pointermove',e=>{
    if(!state||busy)return;const p=localPoint(e);
    if(!gesture){
      const hits=sourceOnly()?[]:pickLines(p,visibleGeometry(),view);
      const next=hits[0]?.id||null;
      if(hover!==next){hover=next;svg.style.cursor=next?'pointer':'grab';render();}
      return;
    }
    if(gesture.kind==='focus')return;
    if(Math.hypot(p[0]-gesture.start[0],p[1]-gesture.start[1])<3&&!gesture.moved)return;
    gesture.moved=true;
    if(gesture.kind==='pan'){
      cameraMode='manual';view={...gesture.view,x:gesture.view.x+p[0]-gesture.start[0],y:gesture.view.y+p[1]-gesture.start[1]};render();
    }else{
      gesture.value=dragValue(gesture.startValue,gesture.start,p,selected.normal,view.k);
      requestPreview(gesture.value);
    }
  });
  svg.addEventListener('pointerup',e=>{
    const g=gesture;gesture=null;if(svg.hasPointerCapture(e.pointerId))svg.releasePointerCapture(e.pointerId);
    if(g?.kind==='handle'&&g.moved)requestPreview(g.value,true);
    else if(g?.kind==='pan'&&!g.moved&&!sourceOnly())picked(localPoint(e));
    render();updateButtons();
  });
  svg.addEventListener('pointercancel',()=>{gesture=null;cancel();});
  svg.addEventListener('dblclick',e=>{if(!sourceOnly()&&selected){focusSelected();e.preventDefault();}});
  function zoom(point,factor){if(!state||gesture)return;cameraMode='manual';view=zoomAt(view,point,factor);render();}
  svg.addEventListener('wheel',e=>{e.preventDefault();zoom(localPoint(e),Math.exp(-e.deltaY*.0015));},{passive:false});
  $('zoomIn').onclick=()=>zoom([svg.clientWidth/2,svg.clientHeight/2],1.5);
  $('zoomOut').onclick=()=>zoom([svg.clientWidth/2,svg.clientHeight/2],1/1.5);
  for(const id of ['sourceEdges','sourcePaths','baseline','candidate','centers','connectors'])$(id).onchange=render;
  $('modeOverlay').onclick=()=>setMode('overlay');$('modeSource').onclick=()=>setMode('source');$('modeResult').onclick=()=>setMode('result');
  $('road').onchange=()=>{choose(null);fit($('road').value);updateHandleList();closePanels();};
  $('fitRoad').onclick=()=>{fit($('road').value);closePanels();};
  for(const id of ['fitAll','homeView'])$(id).onclick=()=>{fit();closePanels();};
  $('focusSelected').onclick=focusSelected;
  $('toggleLeft').onclick=()=>togglePanel('leftPanel');$('toggleRight').onclick=()=>togglePanel('rightPanel');
  $('panelShade').onclick=closePanels;$('closeLeft').onclick=closePanels;$('closeRight').onclick=closePanels;
  $('preview').onclick=()=>requestPreview(Number($('delta').value),true);
  $('delta').oninput=()=>{queue.cancel();clearTimeout(previewTimer);previewTimer=null;preview=null;intent=Number($('delta').value);status('数值已改变，点击“更新预览”或按 Enter 计算。');updateButtons();render();};
  for(const [id,step] of [['minusCm',-.01],['plusCm',.01]])$(id).onclick=()=>requestPreview(Number($('delta').value)+step,true);
  const apply=()=>work(async()=>{if(!preview)return;refresh(await action('apply',{preview_id:preview.preview_id}));status('已应用到副本；Ctrl+Z 可撤销，Ctrl+S 保存项目。');});
  $('apply').onclick=apply;$('applyDock').onclick=apply;
  $('discard').onclick=cancel;$('cancelDock').onclick=cancel;
  for(const name of ['undo','redo'])$(name).onclick=()=>work(async()=>{refresh(await action(name));status(name==='undo'?'已撤销；Ctrl+Y 可重做。':'已重做。');});
  $('save').onclick=()=>work(async()=>{const p=await action('save');state.saved_revision=p.saved_revision;refresh(state);status('项目已保存，包含当前副本与撤销记录。');});
  $('reopen').onclick=()=>$('restoreDialog').showModal();
  $('cancelRestore').onclick=()=>$('restoreDialog').close();
  $('confirmRestore').onclick=()=>{$('restoreDialog').close();work(async()=>{refresh(await action('reopen'));status('已核对原件哈希并恢复磁盘保存版本。');});};
  $('export').onclick=()=>work(async()=>{
    status('写出候选并独立检查 XML / esmini…');exportResult=await action('export');
    const r=exportResult.report;
    $('exportResult').textContent='候选已落盘，非交付\n内部边缘失败 '+r.written_internal_failures+' 项（基线 '+r.baseline_internal_failures+
      '）\nXSD '+(r.consumer?.xsd??'未运行')+' · esmini '+r.esmini+'\n'+exportResult.path+'\nSHA '+r.xodr_sha256.slice(0,16)+'…';
    $('download').hidden=false;$('checks').open=true;
    if(matchMedia('(max-width:1050px)').matches)togglePanel('rightPanel');
    status('候选与报告已落盘，整图仍 BLOCKED。');
  });
  $('download').onclick=()=>work(async()=>{
    const r=await fetch(exportResult.download,{headers:{Authorization:'Bearer '+token}});if(!r.ok)throw new Error('下载失败');
    const url=URL.createObjectURL(await r.blob()),a=document.createElement('a');a.href=url;a.download='node4-repair-CANDIDATE.xodr';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  });
  document.addEventListener('keydown',e=>{
    if(!state)return;
    const typing=e.target.matches('input,select,textarea');
    if(e.key==='Escape'){
      if($('restoreDialog').open)return;closePanels();$('pickMenu').hidden=true;if(!busy)cancel();return;
    }
    if(typing){if(e.key==='Enter'&&e.target===$('delta')){e.preventDefault();requestPreview(Number($('delta').value),true);}return;}
    if(e.ctrlKey||e.metaKey){
      const id=e.key.toLowerCase()==='z'?(e.shiftKey?'redo':'undo'):e.key.toLowerCase()==='y'?'redo':e.key.toLowerCase()==='s'?'save':null;
      if(id){e.preventDefault();if(!$(id).disabled)$(id).click();}return;
    }
    if(e.target!==svg&&e.target!==document.body)return;
    if(e.key.toLowerCase()==='f'){e.preventDefault();focusSelected();}
    else if(e.key==='Home'){e.preventDefault();fit();}
    else if(e.key==='Enter'&&!$('applyDock').disabled){e.preventDefault();apply();}
    else if(e.code==='Space'){e.preventDefault();sourcePeek=true;render();}
    else if(e.key==='Tab'&&e.target===svg){
      const hs=state.handles.filter(h=>h.road===$('road').value);if(!hs.length)return;
      e.preventDefault();let i=hs.findIndex(h=>h.id===selected?.id);i=(i+(e.shiftKey?-1:1)+hs.length)%hs.length;choose(hs[i]);
    }
  });
  document.addEventListener('keyup',e=>{if(e.code==='Space'){sourcePeek=false;render();}});
  window.addEventListener('blur',()=>{sourcePeek=false;if(gesture)cancel();render();});
  new ResizeObserver(()=>{
    const next=[svg.clientWidth,svg.clientHeight],old=size;size=next;
    if(!state||next[0]===old[0]&&next[1]===old[1])return;
    if(cameraMode==='full')fit();
    else if(cameraMode==='road')fit($('road').value);
    else if(cameraMode==='detail')focusSelected();
    else{view=resizeView(view,old,next);render();}
    if(!matchMedia('(max-width:1050px)').matches)closePanels();
  }).observe(svg);
  await work(async()=>{
    project=await api('/api/project');state=project.state;$('projectName').textContent=project.name;
    for(const r of state.roads.filter(r=>!r.junction)){
      const o=document.createElement('option');o.value=r.id;o.textContent='道路 '+r.id+(state.handles.some(h=>h.road===r.id)?' · 可选分界线':' · 锁定');$('road').append(o);
    }
    $('road').value='11';refresh(state);fit();
    status(state.selection?'点青色分界线选中，再放大拖动双箭头柄。':'旧会话仍保留。线选中需使用新版服务；本页保留旧手柄操作。');
  });
}
if(typeof document!=='undefined')await mountWorkbench();
