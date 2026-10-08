"use strict";
const $ = id => document.getElementById(id);
const fragment = new URLSearchParams(location.hash.slice(1));
if (fragment.has("token")) { sessionStorage.setItem("mapforge-workbench-token", fragment.get("token")); history.replaceState(null, "", location.pathname); }
const token = sessionStorage.getItem("mapforge-workbench-token") || "";
let project = null, selected = null, catalog = null, generation = 0, visibleLimit = 150;
let features = [], byId = new Map(), visibleRoles = new Set(), busy = false, currentJob = null;
let view = {x:0,y:0,scale:1}, pointer = null, pendingNote = false;
const canvas = $("map"), ctx = canvas.getContext("2d");
const roleNames = {junction:"路口面",road:"道路",road_center:"道路中心",lane:"车道",lane_merge:"渐变车道",boundary:"车道边界",lane_boundary_rel:"边界关系",topo:"拓扑",stop_line:"停止线"};
const colors = {junction:"#426177",road:"#607d9a",road_center:"#768fbd",lane:"#59a3be",lane_merge:"#ac85bb",boundary:"#82cdb1",stop_line:"#edc57d",topo:"#ba9090"};
function notify(text, error=false) { $("notice").textContent=text; $("notice").classList.toggle("error",error); }
async function api(path, body) {
  const response = await fetch("/api"+path,{method:body===undefined?"GET":"POST",headers:{Authorization:"Bearer "+token,...(body===undefined?{}:{"Content-Type":"application/json","X-Mapforge-CSRF":token})},...(body===undefined?{}:{body:JSON.stringify(body)})});
  const result=await response.json(); if(!response.ok) throw new Error(typeof result.detail==="string"?result.detail:JSON.stringify(result.detail)); return result;
}
async function guarded(action) { if(busy)return; busy=true; setButtons(); try{await action();}catch(e){notify(e.message,true);}finally{busy=false;setButtons();} }
function setButtons() {
  const locked=!project||busy||project.status?.read_only;
  $("save-note").disabled=locked||!selected||!$("note").value.trim(); $("note").disabled=locked||!selected;
  $("undo").disabled=locked||!project.status?.can_undo; $("redo").disabled=locked||!project.status?.can_redo; $("reload").disabled=!project||busy; $("check-source").disabled=!project||busy;
  $("locate-selection").disabled=!selected;
  $("open-editing").disabled=!project||busy;
  $("new-project").disabled=busy||!catalog; $("projects").disabled=busy;
  window.Inspection?.buttons();
  window.SurfaceDiagnostics?.buttons();
}
function make(tag,text,cls) { const e=document.createElement(tag); if(text!==undefined)e.textContent=text; if(cls)e.className=cls; return e; }
function idOf(o) { return String(o.id); }
function roleOf(o) { return o.role||o.layer||o.source_ref?.layer||"unknown"; }
function pointsOf(o) { return (o.points||[]).filter(p=>Array.isArray(p)&&Number.isFinite(p[0])&&Number.isFinite(p[1])); }
function activeFeatures() { return features.filter(o=>visibleRoles.has(roleOf(o))); }
function toScreen(p) { return [(p[0]-view.x)*view.scale+canvas.clientWidth/2,canvas.clientHeight/2-(p[1]-view.y)*view.scale]; }
function fromScreen(x,y) { return [(x-canvas.clientWidth/2)/view.scale+view.x,(canvas.clientHeight/2-y)/view.scale+view.y]; }
function fit(objects=activeFeatures()) {
  let xmin=Infinity,ymin=Infinity,xmax=-Infinity,ymax=-Infinity;
  for(const o of objects)for(const p of pointsOf(o)){xmin=Math.min(xmin,p[0]);ymin=Math.min(ymin,p[1]);xmax=Math.max(xmax,p[0]);ymax=Math.max(ymax,p[1]);}
  if(!Number.isFinite(xmin))return;
  view={x:(xmin+xmax)/2,y:(ymin+ymax)/2,scale:Math.min(Math.max(50,canvas.clientWidth-90)/Math.max(xmax-xmin,1e-8),Math.max(50,canvas.clientHeight-90)/Math.max(ymax-ymin,1e-8))}; draw();
}
function draw() {
  const ratio=devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
  ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
  ctx.strokeStyle="#1c2c3d";ctx.lineWidth=1;ctx.beginPath();for(let x=0;x<w;x+=50){ctx.moveTo(x,0);ctx.lineTo(x,h);}for(let y=0;y<h;y+=50){ctx.moveTo(0,y);ctx.lineTo(w,y);}ctx.stroke();
  const ordered=activeFeatures(); if(selected){const i=ordered.findIndex(o=>idOf(o)===selected);if(i>=0)ordered.push(...ordered.splice(i,1));}
  for(const o of ordered){const pts=pointsOf(o);if(!pts.length)continue;const chosen=idOf(o)===selected,role=roleOf(o);ctx.strokeStyle=chosen?"#ffe499":colors[role]||"#7c93a9";ctx.lineWidth=chosen?3:role==="boundary"?1.5:1;ctx.beginPath();pts.forEach((p,i)=>{const q=toScreen(p);if(i===0)ctx.moveTo(q[0],q[1]);else ctx.lineTo(q[0],q[1]);});if(pts.length===1){const q=toScreen(pts[0]);ctx.arc(q[0],q[1],chosen?5:2.5,0,Math.PI*2);}if(role==="junction"){ctx.closePath();ctx.fillStyle="#38566d33";ctx.fill();}ctx.stroke();}
  window.Inspection?.draw();
}
function renderObjects() {
  const query=$("search").value.toLowerCase().trim(); const objects=activeFeatures().filter(o=>!query||[o.business_id,idOf(o),roleOf(o),JSON.stringify(o.raw_attributes)].join(" ").toLowerCase().includes(query));
  $("objects").replaceChildren();for(const o of objects.slice(0,visibleLimit)){const b=make("button",String(o.business_id||"无业务 ID"));b.append(make("small",`${roleNames[roleOf(o)]||roleOf(o)} · 原记录 ${o.source_ref?.record_index??"?"} · part ${o.source_ref?.part_index??"空"}`));b.classList.toggle("selected",idOf(o)===selected);b.addEventListener("click",()=>selectObject(idOf(o)));$("objects").append(b);}
  $("object-count").textContent=String(objects.length);$("more-objects").hidden=objects.length<=visibleLimit;
}
function selectObject(id,focus=false) {
  if(pendingNote&&!confirm("未确认的待办尚未保存，是否放弃？"))return;
  const o=byId.get(id);if(!o)return;selected=id;pendingNote=false;$("note").value="";
  window.Inspection?.selectionChanged();
  $("selection").replaceChildren(make("strong",roleNames[roleOf(o)]||roleOf(o)),make("div",`业务 ID：${o.business_id||"无"}`),make("div",`原始记录：${o.source_ref?.record_index??"未知"} / part ${o.source_ref?.part_index??"空几何"}`),make("div",`${pointsOf(o).length} 个原始点 · 只读来源`));
  $("raw-attributes").textContent=JSON.stringify(o.raw_attributes||{},null,2);if(focus){visibleRoles.add(roleOf(o));fit([o]);}renderObjects();draw();setButtons();
}
function renderPanels() {
  const snapshot=project.source_snapshot;$("notes").replaceChildren();const intents=project.intents||[];
  for(const intent of intents){const text=intent.type==="shared_boundary_c2_normal_delta"?`共享边界目标：${(intent.parameters.normal_delta_m*1000).toFixed(2)} mm（草稿）`:intent.parameters?.text||intent.text||JSON.stringify(intent);const item=make("div",text,"note-item");$("notes").append(item);}$("note-count").textContent=String(intents.length);
  $("issues").replaceChildren();const issues=[...(snapshot.issues||[]),...(project.status?.source_issues||[])];for(const issue of issues.slice(0,100)){const row=make("div",issue.message||issue.detail||`来源文件需要复核：${issue.relative_path||issue.code||"未知问题"}`,"issue");const refs=issue.object_ids||issue.feature_ids||[];const id=refs.find(x=>byId.has(x));if(id){const b=make("button","定位对象");b.onclick=()=>selectObject(id,true);row.append(b);}$("issues").append(row);}$("issue-count").textContent=String(issues.length);
  const frame=snapshot.frame||{};$("frame-state").textContent=`原坐标 ${frame.coordinate_unit||"未知单位"} · 绝对 CRS：${frame.absolute_crs_status==="unverified"?"未核验":frame.absolute_crs_status||"未知"}`;
  $("save-state").textContent=project.status?.read_only?(project.status.recovery?"恢复副本 · 只读":"来源需复核 · 只读"):`已保存 · 修订 ${project.revision}`;
  $("candidate-state").textContent=project.candidate?(project.status.candidate_stale?"旧候选已过期；草稿需重新编译并接受。":"已有与当前草稿绑定的候选；正式交付仍受完整门禁限制。") :"尚无已接受候选。局部修补通过后仍须整图检查。";
}
function applyProject(data, reset=false) {
  project=data;features=data.source_snapshot.objects||[];byId=new Map(features.map(o=>[idOf(o),o]));
  if(reset){selected=null;currentJob=null;$("job-status").textContent="没有运行中的任务";$("cancel-job").hidden=true;visibleLimit=150;visibleRoles=new Set(features.map(roleOf));$("layers").replaceChildren();for(const role of visibleRoles){const label=make("label"),box=document.createElement("input");box.type="checkbox";box.checked=true;box.onchange=()=>{box.checked?visibleRoles.add(role):visibleRoles.delete(role);renderObjects();draw();};label.append(box,document.createTextNode(roleNames[role]||role));$("layers").append(label);}$("selection").textContent="在地图或列表中选择一个源对象。";$("raw-attributes").textContent="尚未选择对象";$("note").value="";pendingNote=false;}
  $("empty").hidden=true;renderObjects();renderPanels();setButtons();if(reset)fitJunction();else draw();
  if(window.Editor)window.Editor.changed(reset);
  window.Inspection?.changed(reset);
  window.SurfaceDiagnostics?.changed(reset);
}
async function refreshProjects() { const result=await api("/projects");const rows=Array.isArray(result)?result:(result.projects||[]);$("projects").replaceChildren(make("option","打开工程…"));$("projects").firstChild.value="";for(const p of rows){const option=make("option",p.name);option.value=p.project_id;$("projects").append(option);}if(project)$("projects").value=project.project_id; }
async function openProject(id) { if(pendingNote&&!confirm("未确认的待办尚未保存，是否放弃？"))return;const g=++generation;const data=await api("/projects/"+encodeURIComponent(id));if(g!==generation)return;currentJob=null;$("job-status").textContent="没有运行中的任务";$("cancel-job").hidden=true;applyProject(data,true);await refreshJobList();notify("工程已打开。可查看源对象，或打开边界修补核对此工程的编辑范围。"); }
function showImport() { if(!catalog)return;$("import-dialog").showModal(); }
$("new-project").onclick=showImport;$("empty-import").onclick=showImport;$("cancel-import").onclick=()=>$("import-dialog").close();
$("junctions").onchange=()=>{$("project-name").value="路口 "+$("junctions").value;};
$("import-form").onsubmit=e=>{e.preventDefault();guarded(async()=>{if(pendingNote&&!confirm("放弃未确认待办并建立新工程？"))return;notify("正在读取源对象并保存工程…");const data=await api("/projects",{junction_id:$("junctions").value,name:$("project-name").value});generation++;applyProject(data,true);await refreshProjects();$("import-dialog").close();notify("源工程已建立并保存。没有运行整图转换，也没有修改原件。");});};
$("projects").onchange=()=>{if($("projects").value)guarded(()=>openProject($("projects").value));};
$("reload").onclick=()=>guarded(()=>openProject(project.project_id));
$("note").oninput=()=>{pendingNote=Boolean($("note").value);$("save-state").textContent=pendingNote?"待办输入尚未确认":`已保存 · 修订 ${project.revision}`;setButtons();};
$("save-note").onclick=()=>guarded(async()=>{const data=await api(`/projects/${project.project_id}/commands`,{base_revision:project.revision,command:{command_id:crypto.randomUUID(),type:"annotation",source_ref:selected,scope:{feature_ids:[selected]},parameters:{text:$("note").value.trim(),status:"unresolved"}}});pendingNote=false;$("note").value="";applyProject(data);notify("待办已写入工程，可撤销、重做和重开；原件与几何未改变。");});
for(const action of ["undo","redo"])$(action).onclick=()=>guarded(async()=>{if(pendingNote&&!confirm("放弃未确认待办并继续？"))return;const data=await api(`/projects/${project.project_id}/${action}`,{base_revision:project.revision,command_id:crypto.randomUUID()});pendingNote=false;$("note").value="";applyProject(data);notify(action==="undo"?"已撤销一个草稿事务并保存；原候选需核对是否过期。":"已重做一个草稿事务并保存；原候选需核对是否过期。");});
$("check-source").onclick=()=>guarded(async()=>{const id=project.project_id,g=generation;currentJob=await api(`/projects/${id}/source-check`,{base_revision:project.revision,request_id:crypto.randomUUID()});$("cancel-job").hidden=false;pollJob(id,currentJob.job_id,g);});
let jobSelection=0;
const taskNames={source_check:"来源复核",compile:"候选编译",validate:"整图检查"};
async function refreshJobList(){
  if(!project)return;const id=project.project_id,g=generation,selection=++jobSelection;
  try{const rows=await api(`/projects/${id}/jobs`);if(g!==generation||project.project_id!==id||selection!==jobSelection)return;
    $("job-history").replaceChildren();for(const job of rows){const option=make("option",`${taskNames[job.operation]||job.operation} · ${new Date(job.started_at*1000).toLocaleTimeString()} · ${job.historical?"历史":job.state}`);option.value=job.job_id;$("job-history").append(option);}
    $("job-history").disabled=!rows.length;
    if(rows.length){const job=rows.find(x=>x.state==="running")||rows[0];$("job-history").value=job.job_id;pollJob(id,job.job_id,g,selection);}
    else{$("job-history").append(make("option","尚无任务"));$("job-status").textContent="没有运行中的任务";$("cancel-job").hidden=true;}
  }catch(e){if(g===generation)$("job-status").textContent="任务记录暂不可读："+e.message;}
}
$("job-history").onchange=()=>{if(project)pollJob(project.project_id,$("job-history").value,generation,++jobSelection);};
async function pollJob(projectId,jobId,g,selection=++jobSelection){
  if(g!==generation||selection!==jobSelection)return;
  try{const job=await api(`/projects/${projectId}/jobs/${jobId}`);if(g!==generation||selection!==jobSelection)return;currentJob=job;
    let success=job.result?.matches?"源文件与工程快照一致；不代表地图质量通过。":`来源发生变化（${job.result?.issues?.length||0} 项），请重开工程查看。`;
    if(job.operation==="compile")success=job.result?.status==="COMPILED"?"候选已生成；接受状态以工程记录为准。":"候选拒绝："+(job.result?.error?.message||"未通过检查");
    if(job.operation==="validate")success=job.result?.status==="VALIDATED"?"检查完成 · 交付裁决 "+job.result.validation.decision:"检查拒绝："+(job.result?.error?.message||"未完成");
    const labels={running:`${taskNames[job.operation]||"后台任务"}运行中，工程可继续查看…`,cancelled:"任务已取消",timed_out:"任务超时",orphaned:"历史任务状态未确认，未自动重新启动",failed:"任务失败："+(job.error||"工作进程异常"),succeeded:success};
    $("job-status").textContent=(job.historical?"历史记录 · ":job.stale?"对应较早修订 · ":"")+(labels[job.state]||job.state)+(job.archive?.state==="failed"?" · 结果未持久化，请检查磁盘。":"");
    $("cancel-job").hidden=job.state!=="running"||job.historical;
    if(job.state==="running")setTimeout(()=>pollJob(projectId,jobId,g,selection),750);
  }catch(e){if(g===generation&&selection===jobSelection){$("job-status").textContent="任务状态暂不可读取："+e.message+"；继续查询同一任务";setTimeout(()=>pollJob(projectId,jobId,g,selection),2000);}}
}
$("cancel-job").onclick=()=>guarded(async()=>{if(currentJob)await api(`/projects/${project.project_id}/jobs/${currentJob.job_id}/cancel`,{});});
$("search").oninput=()=>{visibleLimit=150;renderObjects();};$("more-objects").onclick=()=>{visibleLimit+=150;renderObjects();};$("fit").onclick=()=>fit();$("zoom-in").onclick=()=>{view.scale*=1.4;draw();};$("zoom-out").onclick=()=>{view.scale/=1.4;draw();};
function fitJunction(){const junctions=features.filter(o=>roleOf(o)==="junction"&&pointsOf(o).length);fit(junctions.length?junctions:activeFeatures());view.scale*=0.55;draw();}
$("fit-junction").onclick=fitJunction;$("locate-selection").onclick=()=>{if(selected)fit([byId.get(selected)]);};
function segmentDistance(p,a,b){const dx=b[0]-a[0],dy=b[1]-a[1],d=dx*dx+dy*dy,t=d?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d)):0;return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);}
function pick(x,y){const hits=[];for(const o of activeFeatures()){const pts=pointsOf(o).map(toScreen);let distance=Infinity;for(let i=0;i<pts.length;i++)distance=Math.min(distance,segmentDistance([x,y],pts[i],pts[Math.min(i+1,pts.length-1)]));if(distance<10)hits.push({id:idOf(o),distance});}hits.sort((a,b)=>a.distance-b.distance||a.id.localeCompare(b.id));if(window.Inspection)window.Inspection.pick(hits);else if(hits.length)selectObject(hits[0].id);}
canvas.addEventListener("pointerdown",e=>{pointer={x:e.offsetX,y:e.offsetY,startX:e.offsetX,startY:e.offsetY,moved:false};canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener("pointermove",e=>{const p=fromScreen(e.offsetX,e.offsetY);$("coordinates").textContent=`x ${p[0].toFixed(7)} · y ${p[1].toFixed(7)}`;if(pointer){const dx=e.offsetX-pointer.x,dy=e.offsetY-pointer.y;if(Math.hypot(e.offsetX-pointer.startX,e.offsetY-pointer.startY)>4)pointer.moved=true;if(pointer.moved){view.x-=dx/view.scale;view.y+=dy/view.scale;draw();}pointer.x=e.offsetX;pointer.y=e.offsetY;}});
canvas.addEventListener("pointerup",e=>{if(pointer&&!pointer.moved&&!window.Inspection?.point(fromScreen(e.offsetX,e.offsetY)))pick(e.offsetX,e.offsetY);pointer=null;});canvas.addEventListener("pointercancel",()=>pointer=null);
canvas.addEventListener("wheel",e=>{e.preventDefault();const before=fromScreen(e.offsetX,e.offsetY);view.scale*=Math.exp(-Math.max(-150,Math.min(150,e.deltaY))*.002);const after=fromScreen(e.offsetX,e.offsetY);view.x+=before[0]-after[0];view.y+=before[1]-after[1];draw();},{passive:false});
canvas.addEventListener("keydown",e=>{if(e.key.toLowerCase()==="f")fit();});new ResizeObserver(()=>draw()).observe(canvas);
window.addEventListener("beforeunload",e=>{if(pendingNote){e.preventDefault();e.returnValue="";}});
document.addEventListener("keydown",e=>{if($("editing-dialog").open||$("import-dialog").open||$("diagnostics-dialog").open){if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==="s")e.preventDefault();return;}if(e.key==="Escape"&&window.Inspection?.escape()){e.preventDefault();return;}if(e.key==="Escape"&&pendingNote){$("note").value="";pendingNote=false;renderPanels();setButtons();}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==="s"){e.preventDefault();if(pendingNote)notify("待办输入尚未确认，请使用“确认并保存待办”。");else if(project)notify("已确认的工程事务已保存到本机。");}});
guarded(async()=>{catalog=await api("/catalog");for(const j of catalog.junctions){const option=make("option",`${j.name||"路口"} · ${j.id}`);option.value=j.id;$("junctions").append(option);}$("project-name").value="路口 "+$("junctions").value;await refreshProjects();notify(catalog.notice);});
