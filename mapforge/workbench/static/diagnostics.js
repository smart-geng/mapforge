"use strict";
// A separate local-metre view of a bound research report. Reading or locating a
// diagnostic never changes the project, its draft, or its accepted candidate.
window.SurfaceDiagnostics=(()=>{
  const panel=$("diagnostics-dialog"),plot=$("diagnostics-map"),pen=plot.getContext("2d");
  const palette=["#55c8c0","#8fa3ba","#b49ade","#78b98b"];
  let report=null,selected=null,sequence=0,owner=null,drag=null,view={x:0,y:0,scale:1};
  const text=value=>typeof value==="string"?value:"";
  const status=value=>$("diagnostics-status").textContent=value;
  function buttons(){$("open-diagnostics").disabled=!project||busy;}
  function controls(){for(const id of ["diagnostics-fit","diagnostics-zoom-in","diagnostics-zoom-out"])$(id).disabled=!report;}
  function polygons(geometry){if(geometry?.type==="Polygon")return [geometry.coordinates];if(geometry?.type==="MultiPolygon")return geometry.coordinates;return [];}
  function paths(geometry){if(geometry?.type==="LineString")return [geometry.coordinates];if(geometry?.type==="MultiLineString")return geometry.coordinates;return polygons(geometry).flat();}
  function validGeometry(geometry){
    if(!geometry||!["Polygon","MultiPolygon","LineString","MultiLineString"].includes(geometry.type))return false;
    try{const lines=paths(geometry);return Array.isArray(lines)&&lines.length>0&&lines.every(line=>Array.isArray(line)&&line.length>=2&&line.every(p=>Array.isArray(p)&&p.length>=2&&Number.isFinite(p[0])&&Number.isFinite(p[1])));}catch{return false;}
  }
  function validReport(value){
    const frame=value?.context?.frame;
    return frame?.kind==="local-eqc"&&frame.unit==="m"&&frame.absolute_crs_status==="unverified"&&
      Array.isArray(frame.origin)&&frame.origin.length===2&&frame.origin.every(Number.isFinite)&&
      value.context.candidate_accepted===false&&value.context.formal_release_verified===false&&
      Array.isArray(value.layers)&&value.layers.every(layer=>validGeometry(layer.geometry))&&
      Array.isArray(value.issues)&&value.issues.every(issue=>typeof issue.id==="string"&&validGeometry(issue.geometry))&&
      new Set(value.issues.map(issue=>issue.id)).size===value.issues.length;
  }
  function screen(p){return [(p[0]-view.x)*view.scale+plot.clientWidth/2,plot.clientHeight/2-(p[1]-view.y)*view.scale];}
  function world(x,y){return [(x-plot.clientWidth/2)/view.scale+view.x,(plot.clientHeight/2-y)/view.scale+view.y];}
  function trace(line,close){line.forEach((p,i)=>{const q=screen(p);if(i)pen.lineTo(...q);else pen.moveTo(...q);});if(close)pen.closePath();}
  function shape(geometry,color,alpha,width){
    pen.strokeStyle=color;pen.fillStyle=color;pen.lineWidth=width;
    const areas=polygons(geometry);
    if(areas.length){
      // Every polygon has its own path: inner rings stay empty regardless of
      // winding, while disconnected MultiPolygon members remain independent.
      for(const area of areas){pen.beginPath();for(const ring of area)trace(ring,true);pen.globalAlpha=alpha;pen.fill("evenodd");pen.globalAlpha=1;pen.stroke();}
    }else{pen.beginPath();for(const line of paths(geometry))trace(line,false);pen.stroke();}
  }
  function paint(){
    const ratio=devicePixelRatio||1,w=plot.clientWidth,h=plot.clientHeight;
    if(plot.width!==Math.round(w*ratio)||plot.height!==Math.round(h*ratio)){plot.width=Math.round(w*ratio);plot.height=Math.round(h*ratio);}
    pen.setTransform(ratio,0,0,ratio,0,0);pen.clearRect(0,0,w,h);if(!report)return;
    report.layers.forEach((layer,i)=>shape(layer.geometry,palette[i%palette.length],.12,1));
    for(const issue of report.issues)if(issue.id!==selected)shape(issue.geometry,"#ffbd65",.25,1.5);
    const active=report.issues.find(issue=>issue.id===selected);if(active)shape(active.geometry,"#ff8b72",.45,2.5);
    pen.strokeStyle="#b9cbdc";pen.lineWidth=2;pen.beginPath();pen.moveTo(20,h-25);pen.lineTo(120,h-25);pen.stroke();
    pen.fillStyle="#b9cbdc";pen.font="12px sans-serif";pen.fillText(`${(100/view.scale).toPrecision(3)} m`,20,h-35);
  }
  function fit(geometries){
    let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;
    for(const geometry of geometries)for(const line of paths(geometry))for(const p of line){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}
    if(Number.isFinite(x0))view={x:(x0+x1)/2,y:(y0+y1)/2,scale:Math.min(Math.max(100,plot.clientWidth-100)/Math.max(1,x1-x0),Math.max(100,plot.clientHeight-100)/Math.max(1,y1-y0))};
    paint();
  }
  function fitAll(){if(report)fit([...report.layers,...report.issues].map(row=>row.geometry));}
  function issueDetails(issue){
    const lines=[text(issue.detail)];
    if(Number.isFinite(issue.area_m2))lines.push(`面积 ${issue.area_m2.toPrecision(5)} m²`);
    if(Array.isArray(issue.road_ids)&&issue.road_ids.length)lines.push("道路 ID："+issue.road_ids.join("、"));
    if(Array.isArray(issue.source_lane_ids)&&issue.source_lane_ids.length)lines.push("来源车道 ID："+issue.source_lane_ids.join("、"));
    return lines.filter(Boolean).join("\n");
  }
  function select(id){
    const issue=report?.issues.find(item=>item.id===id);if(!issue)return;selected=id;
    for(const button of $("diagnostics-issues").children)button.setAttribute("aria-pressed",String(button.dataset.issueId===id));
    $("diagnostics-detail").textContent=[text(issue.title),issueDetails(issue)].filter(Boolean).join("\n");fit([issue.geometry]);
  }
  function show(){
    const list=$("diagnostics-issues"),legend=$("diagnostics-legend");list.replaceChildren();legend.replaceChildren();
    report.layers.forEach((layer,i)=>{const item=make("span",text(layer.title)||"诊断图层");item.style.borderLeftColor=palette[i%palette.length];legend.append(item);});
    for(const issue of report.issues){const button=make("button",text(issue.title)||text(issue.kind)||issue.id);button.dataset.issueId=issue.id;button.setAttribute("aria-pressed","false");button.append(make("small",issueDetails(issue)));button.onclick=()=>select(issue.id);list.append(button);}
    if(!report.issues.length)list.append(make("p","报告未列出可定位的问题。这不代表整图或正式交付通过。"));
    const context=report.context;
    $("diagnostics-binding").textContent=`研究候选 SHA256：${text(context.candidate_sha256)}\n来源快照：${text(context.source_snapshot_id)}\n来源内容 SHA256：${text(context.source_content_hash)}`;
    $("diagnostics-frame").textContent=`局部等距圆柱坐标 · 单位 m · 原点 ${context.frame.origin.join(", ")} · 绝对 CRS 未核验`;
    $("diagnostics-coordinates").textContent="局部米制坐标 · 绝对位置未核验";
    status(`已载入 ${report.issues.length} 项诊断。点击问题定位；研究候选未接受，正式交付仍阻断。`);controls();fitAll();
  }
  function clear(){report=null;selected=null;drag=null;view={x:0,y:0,scale:1};$("diagnostics-issues").replaceChildren();$("diagnostics-legend").replaceChildren();$("diagnostics-detail").textContent="";$("diagnostics-binding").textContent="";$("diagnostics-frame").textContent="局部米制坐标；绝对 CRS 未核验";$("diagnostics-coordinates").textContent="尚无诊断几何";controls();paint();}
  async function open(){
    if(!project||busy)return;
    const id=project.project_id,g=generation,revision=project.revision,ticket=++sequence;owner=id;clear();status("正在核对来源与研究候选证据…");if(!panel.open)panel.showModal();
    const current=()=>panel.open&&sequence===ticket&&owner===id&&project?.project_id===id&&generation===g&&project.revision===revision;
    try{
      const response=await api(`/projects/${encodeURIComponent(id)}/surface-diagnostics`);if(!current())return;
      if(response.available!==true){status(text(response.reason)||"此工程暂无匹配的研究诊断；画布保持为空。");return;}
      if(!validReport(response.report)){status("诊断坐标框架或内容不完整，未显示几何。请重新核对研究证据。");return;}
      report=response.report;show();
    }catch(error){if(current())status("诊断读取失败："+error.message);}
  }
  function close(){sequence++;owner=null;clear();if(panel.open)panel.close();status("诊断已关闭，未修改工程。");}
  function changed(){sequence++;owner=null;clear();if(panel.open)panel.close();buttons();}
  function zoom(factor){if(!report)return;view.scale=Math.max(1e-8,Math.min(1e8,view.scale*factor));paint();}
  $("open-diagnostics").onclick=open;$("close-diagnostics").onclick=close;
  $("diagnostics-fit").onclick=fitAll;$("diagnostics-zoom-in").onclick=()=>zoom(1.5);$("diagnostics-zoom-out").onclick=()=>zoom(1/1.5);
  panel.addEventListener("cancel",event=>{event.preventDefault();close();});
  panel.addEventListener("close",()=>{if(panel.open)return;sequence++;owner=null;clear();});
  plot.onpointerdown=event=>{if(!report)return;drag={x:event.offsetX,y:event.offsetY};plot.setPointerCapture(event.pointerId);};
  plot.onpointermove=event=>{if(!report)return;const p=world(event.offsetX,event.offsetY);$("diagnostics-coordinates").textContent=`局部 x ${p[0].toFixed(3)} m · y ${p[1].toFixed(3)} m`;if(drag){view.x-=(event.offsetX-drag.x)/view.scale;view.y+=(event.offsetY-drag.y)/view.scale;drag={x:event.offsetX,y:event.offsetY};paint();}};
  plot.onpointerup=plot.onpointercancel=plot.onlostpointercapture=()=>{drag=null;};
  plot.addEventListener("wheel",event=>{if(!report)return;event.preventDefault();const a=world(event.offsetX,event.offsetY);zoom(Math.exp(-Math.max(-150,Math.min(150,event.deltaY))*.002));const b=world(event.offsetX,event.offsetY);view.x+=a[0]-b[0];view.y+=a[1]-b[1];paint();},{passive:false});
  new ResizeObserver(paint).observe(plot);buttons();controls();
  return {buttons,changed};
})();
