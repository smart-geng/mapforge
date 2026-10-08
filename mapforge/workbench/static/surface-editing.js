"use strict";
// Only server-owned fresh-source jobs supply this pane's geometry. A preview,
// confirmed draft, accepted research candidate and saved check are distinct.
window.SurfaceEditor=(()=>{
  const TYPE="source_supported_surface_tracks_v2";
  const panel=$("surface-editing-dialog"),plot=$("surface-map"),pen=plot.getContext("2d");
  const palette=["#55c8c0","#9fb4cb","#b9a1df","#83c59b"],sessions=new Map();
  let session=null,owner=null,epoch=0,geometrySequence=0,applying=false,drag=null,view={x:0,y:0,scale:1};
  const clone=value=>JSON.parse(JSON.stringify(value));
  const stable=value=>JSON.stringify(value,(_,item)=>item&&typeof item==="object"&&!Array.isArray(item)?Object.fromEntries(Object.keys(item).sort().map(key=>[key,item[key]])):item);
  const version=()=>stable({id:project?.project_id,revision:project?.revision,epoch:project?.draft_epoch,content:project?.content_hash,context:project?.context});
  const intents=()=>project?.intents?.filter(item=>item.type===TYPE)||[];
  const running=job=>["starting","running","cancelling"].includes(job?.state)&&!job?.historical;
  const anyRunning=()=>running(session?.compileJob)||running(session?.checkJob);
  const persisted=job=>job?.archive?.state==="persisted"&&job.archive.recorded_state==="succeeded"&&!job.historical;
  const candidateCurrent=()=>Boolean(project?.candidate&&!project.status?.candidate_stale&&!project.status?.read_only);
  function freshSession(){return {descriptor:null,ticket:null,compileJob:null,checkJob:null,checkTarget:null,geometry:null,fidelity:null,selectedFidelity:null,mutation:null,requests:{},pollSequence:{compile:0,check:0},status:"尚未计算候选。"};}
  function requestBody(kind,create,identity=""){const bound=version()+identity,previous=session.requests[kind];if(previous?.bound===bound)return clone(previous.body);const body=create();session.requests[kind]={bound,body:clone(body)};return body;}
  function state(value){if(session)session.status=value;$("surface-status").textContent=value;}
  function current(s,id,g){return session===s&&owner===id&&project?.project_id===id&&epoch===g&&panel.open;}
  function buttons(){$("open-surface-editing").disabled=!project||busy;}
  function ticketMatchesDraft(){const ticket=session?.ticket;return Boolean(ticket&&ticket.target_revision===project?.revision&&ticket.target_draft_epoch===project?.draft_epoch&&ticket.target_content_hash===project?.content_hash);}
  function ticketMatchesBase(){const ticket=session?.ticket;return Boolean(ticket&&ticket.base_revision===project?.revision&&ticket.base_epoch===project?.draft_epoch&&ticket.base_content===project?.content_hash&&ticket.base_context===stable(project?.context));}
  function compiled(){return session?.compileJob?.state==="succeeded"&&session.compileJob.result?.status==="COMPILED"&&persisted(session.compileJob);}
  function canAccept(){return compiled()&&ticketMatchesDraft()&&project?.candidate?.candidate_id!==session.compileJob.result?.candidate?.candidate_id;}
  function canAttach(){const job=session?.checkJob;return job?.state==="succeeded"&&job.result?.status==="VALIDATED"&&persisted(job)&&session.checkTarget===version()&&job.base_revision===project?.revision&&job.content_hash===project?.content_hash;}
  function controls(){
    buttons();const locked=!project||busy||Boolean(session?.mutation)||project?.status?.read_only||!session?.descriptor?.enabled;
    const count=intents().length;
    $("surface-enable").hidden=!session?.descriptor?.supported||session.descriptor.enabled;
    $("surface-enable").disabled=!project||busy||Boolean(session?.mutation)||project?.status?.read_only||!session?.descriptor?.supported;
    $("surface-preview").disabled=locked||anyRunning()||count!==0;
    $("surface-save").disabled=locked||count!==0||!ticketMatchesBase();
    $("surface-compile").disabled=locked||anyRunning()||count!==1;
    $("surface-accept").disabled=locked||!canAccept();
    $("surface-check").disabled=locked||anyRunning()||!candidateCurrent();
    $("surface-attach").disabled=locked||!canAttach();
    $("surface-export").disabled=locked||anyRunning()||!candidateCurrent()||!project?.validation||project.status?.validation_stale;
    $("surface-cancel").hidden=!anyRunning();$("surface-cancel").disabled=Boolean(session?.mutation)||busy;
    $("surface-close").disabled=Boolean(session?.mutation);
    for(const id of ["surface-fit","surface-zoom-in","surface-zoom-out"])$(id).disabled=!session?.geometry;
    $("surface-draft").textContent=count===1?"重建草稿已确认保存；可编译当前草稿。":count>1?"存在多个重建意图，请返回源工程撤销重复操作。":"重建意图尚未确认保存。预览不写入草稿。";
  }
  function clearFidelity(reason="尚无本次候选的来源保真定位"){if(session){session.fidelity=null;session.selectedFidelity=null;}$("surface-fidelity-status").textContent=reason;$("surface-fidelity-lanes").replaceChildren();$("surface-fidelity-stops").replaceChildren();$("surface-fidelity-refs").replaceChildren();$("surface-fidelity-primary").textContent="";$("surface-fidelity-selection").textContent="未选中比较车道";$("surface-fidelity-clear").disabled=true;}
  function clearGeometry(){geometrySequence++;if(session)session.geometry=null;clearFidelity();drag=null;$("surface-legend").replaceChildren();$("surface-issues").replaceChildren();$("surface-geometry-state").textContent="尚无本次候选几何";$("surface-binding").textContent="";$("surface-coordinates").textContent="局部米制画布，绝对位置未核验";paint();}
  function paths(geometry){if(geometry?.type==="LineString")return [geometry.coordinates];if(geometry?.type==="MultiLineString")return geometry.coordinates;return polygons(geometry).flat();}
  function polygons(geometry){return geometry?.type==="Polygon"?[geometry.coordinates]:geometry?.type==="MultiPolygon"?geometry.coordinates:[];}
  function validGeometry(geometry){try{return ["Polygon","MultiPolygon","LineString","MultiLineString"].includes(geometry?.type)&&paths(geometry).length>0&&paths(geometry).every(line=>Array.isArray(line)&&line.length>=2&&line.every(p=>Array.isArray(p)&&p.length>=2&&Number.isFinite(p[0])&&Number.isFinite(p[1])));}catch{return false;}}
  function validReport(value,expectedId){
    const report=value?.report,context=report?.context,frame=context?.frame;
    return value.available===true&&value.candidate_stale!==true&&value.candidate_id===expectedId&&context?.candidate_id===expectedId&&context.project_id===owner&&
      frame?.kind==="local-eqc"&&frame.unit==="m"&&frame.absolute_crs_status==="unverified"&&Array.isArray(frame.origin)&&frame.origin.length===2&&frame.origin.every(Number.isFinite)&&
      context.formal_release_verified===false&&Array.isArray(report.layers)&&report.layers.every(layer=>validGeometry(layer.geometry))&&
      Array.isArray(report.issues)&&report.issues.every(issue=>validGeometry(issue.geometry));
  }
  const nonempty=value=>typeof value==="string"&&value.length>0;
  const stopMissing=lane=>lane.issues.some(issue=>issue.code==="stopline-unmeasurable");
  function validSourceRefs(lane){
    const valid=ref=>ref&&nonempty(ref.id)&&["lane","lane_merge"].includes(ref.role)&&nonempty(ref.source_ref?.layer)&&ref.source_ref.snapshot_id===session?.geometry?.report?.context?.source_snapshot_id&&Number.isInteger(ref.source_ref.record_index)&&ref.source_ref.record_index>=0&&(ref.source_ref.part_index===null||Number.isInteger(ref.source_ref.part_index)&&ref.source_ref.part_index>=0);
    const refs=lane.source_refs,primary=lane.primary_source_ref;
    return Array.isArray(refs)&&refs.length>0&&refs.every(valid)&&new Set(refs.map(ref=>ref.id)).size===refs.length&&valid(primary)&&primary.role==="lane"&&primary.source_ref.layer==="IBD_LANE_LINK"&&refs.filter(ref=>stable(ref)===stable(primary)).length===1;
  }
  function validFidelity(value){
    const stats=row=>row&&Number.isInteger(row.count)&&row.count>0&&["p95_m","max_m","median_m"].every(key=>Number.isFinite(row[key])&&row[key]>=0);
    const line=geometry=>geometry?.type==="LineString"&&validGeometry(geometry)&&geometry.coordinates.every(point=>point.length===2);
    return value?.schema==="mapforge/workbench-fidelity-inspection/v1"&&value.status==="AVAILABLE"&&["PASS","FAIL","REVIEW","UNAVAILABLE"].includes(value.gate_status)&&
      value.scope&&typeof value.scope==="object"&&!Array.isArray(value.scope)&&Array.isArray(value.lanes)&&value.lanes.length>0&&
      value.summary?.lane_count===value.lanes.length&&value.scope.matched_source_lanes===value.lanes.length&&new Set(value.lanes.map(lane=>lane?.id)).size===value.lanes.length&&
      value.lanes.every(lane=>lane&&nonempty(lane.id)&&nonempty(lane.source_lane_id)&&nonempty(lane.policy_class)&&nonempty(lane.target?.road_id)&&
        ["left","right"].includes(lane.target.side)&&Number.isInteger(lane.target.lane_id)&&Number.isInteger(lane.target.section_first)&&Number.isInteger(lane.target.section_last)&&lane.target.section_first>=0&&lane.target.section_last>=lane.target.section_first&&
        stats(lane.source_to_target)&&stats(lane.target_to_source)&&nonempty(lane.stop_line?.availability)&&Array.isArray(lane.issues)&&lane.issues.every(issue=>issue&&typeof issue==="object"&&!Array.isArray(issue))&&line(lane.source_geometry)&&line(lane.target_geometry)&&validSourceRefs(lane))&&
      value.summary.stopline_unavailable_count===value.lanes.filter(stopMissing).length;
  }
  function fidelityButtons(){for(const id of ["surface-fidelity-lanes","surface-fidelity-stops"])for(const button of $(id).children)if(button.dataset.laneId)button.setAttribute("aria-pressed",String(button.dataset.laneId===session?.selectedFidelity?.id));}
  function sourceRefLabel(item){const ref=item.source_ref;return `${ref.layer} · 记录 ${ref.record_index} · part ${ref.part_index===null?"无几何":ref.part_index}（索引从 0 开始）`;}
  function selectFidelity(lane){
    if(!panel.open||!session?.fidelity||!session.fidelity.lanes.includes(lane))return;
    session.selectedFidelity=lane;$("surface-fidelity-clear").disabled=false;fidelityButtons();
    $("surface-fidelity-selection").textContent=`来源车道 ${lane.source_lane_id} → 道路 ${lane.target.road_id} / 车道 ${lane.target.lane_id}；${lane.policy_class}。仅叠加当前选中车道的两条完整比较线。`;
    $("surface-fidelity-primary").textContent="主来源记录："+sourceRefLabel(lane.primary_source_ref);$("surface-fidelity-refs").replaceChildren();
    for(const ref of lane.source_refs.filter(ref=>ref.id!==lane.primary_source_ref.id))$("surface-fidelity-refs").append(make("p",sourceRefLabel(ref)+" · "+ref.role+" · 仅补充来源引用，不作为叠图比较线"));
    if(lane.source_refs.length===1)$("surface-fidelity-refs").append(make("p","没有补充来源引用。"));
    fit([lane.source_geometry,lane.target_geometry]);
  }
  const fidelityDistance=value=>value>0&&value<.001?value.toPrecision(3):value.toFixed(3);
  function showFidelity(value){
    clearFidelity();if(value?.status==="UNAVAILABLE"){ $("surface-fidelity-status").textContent=value.reason||"来源保真定位不可用";return;}
    if(!validFidelity(value)){ $("surface-fidelity-status").textContent="来源保真定位不可用：比较对象、数值或几何不完整。铺面诊断仍可查看。";return;}
    session.fidelity=value;const missing=value.lanes.filter(stopMissing),lanes=[...value.lanes].sort((a,b)=>b.source_to_target.p95_m-a.source_to_target.p95_m||a.source_lane_id.localeCompare(b.source_lane_id));
    $("surface-fidelity-status").textContent=`原 G8：${value.gate_status}；共 ${lanes.length} 条比较车道，停止线不可测 ${missing.length} 条。按来源→候选 P95 从大到小排列；此处不判单车道通过或失败。`;
    const buttonFor=(lane,stop=false)=>{const button=make("button",`来源 ${lane.source_lane_id} · 道路 ${lane.target.road_id} / 车道 ${lane.target.lane_id}`);button.dataset.laneId=lane.id;button.setAttribute("aria-pressed","false");
      button.append(make("small",stop?lane.issues.filter(issue=>issue.code==="stopline-unmeasurable").map(issue=>issue.message||issue.reason||"来源停止线不可测").join("；"):`来源→候选 P95 ${fidelityDistance(lane.source_to_target.p95_m)} m · 候选→来源 P95 ${fidelityDistance(lane.target_to_source.p95_m)} m`));
      if(!stop&&stopMissing(lane))button.append(make("small","停止线不可测；请核对原始对象及显式关联。"));button.onclick=()=>selectFidelity(lane);return button;};
    for(const lane of lanes)$("surface-fidelity-lanes").append(buttonFor(lane));
    for(const lane of lanes.filter(stopMissing))$("surface-fidelity-stops").append(buttonFor(lane,true));
    if(!missing.length)$("surface-fidelity-stops").append(make("p","本次比较对象没有停止线不可测项。"));
  }
  function screen(p){return [(p[0]-view.x)*view.scale+plot.clientWidth/2,plot.clientHeight/2-(p[1]-view.y)*view.scale];}
  function world(x,y){return [(x-plot.clientWidth/2)/view.scale+view.x,(plot.clientHeight/2-y)/view.scale+view.y];}
  function trace(line,close){line.forEach((p,index)=>{const q=screen(p);if(index)pen.lineTo(...q);else pen.moveTo(...q);});if(close)pen.closePath();}
  function shape(geometry,color,opacity,width=1.4){pen.strokeStyle=color;pen.fillStyle=color;pen.lineWidth=width;const areas=polygons(geometry);if(areas.length){for(const area of areas){pen.beginPath();for(const ring of area)trace(ring,true);pen.globalAlpha=opacity;pen.fill("evenodd");pen.globalAlpha=1;pen.stroke();}}else{pen.beginPath();for(const line of paths(geometry))trace(line,false);pen.stroke();}}
  function paint(){
    const ratio=devicePixelRatio||1,w=plot.clientWidth,h=plot.clientHeight;
    if(plot.width!==Math.round(w*ratio)||plot.height!==Math.round(h*ratio)){plot.width=Math.round(w*ratio);plot.height=Math.round(h*ratio);}
    pen.setTransform(ratio,0,0,ratio,0,0);pen.clearRect(0,0,w,h);const report=session?.geometry?.report;if(!report)return;
    report.layers.forEach((layer,index)=>shape(layer.geometry,palette[index%palette.length],.14));
    for(const issue of report.issues)shape(issue.geometry,"#ffb76b",.28);
    if(session.selectedFidelity){shape(session.selectedFidelity.source_geometry,"#66baff",1,4.5);shape(session.selectedFidelity.target_geometry,"#ff83d1",1,2.5);}
    pen.strokeStyle="#b9cbdc";pen.lineWidth=2;pen.beginPath();pen.moveTo(20,h-25);pen.lineTo(120,h-25);pen.stroke();pen.fillStyle="#b9cbdc";pen.font="12px sans-serif";pen.fillText(`${(100/view.scale).toPrecision(3)} m`,20,h-35);
  }
  function fit(geometries){let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;for(const geometry of geometries)for(const line of paths(geometry))for(const p of line){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}if(Number.isFinite(x0))view={x:(x0+x1)/2,y:(y0+y1)/2,scale:Math.min(Math.max(100,plot.clientWidth-90)/Math.max(1,x1-x0),Math.max(100,plot.clientHeight-90)/Math.max(1,y1-y0))};paint();}
  function fitAll(){if(session?.geometry)fit([...session.geometry.report.layers,...session.geometry.report.issues].map(item=>item.geometry));}
  function showGeometry(value){
    session.geometry=value;const report=value.report;$("surface-legend").replaceChildren();$("surface-issues").replaceChildren();
    report.layers.forEach((layer,index)=>{const label=make("span",layer.title||"几何图层");label.style.borderLeftColor=palette[index%palette.length];$("surface-legend").append(label);});
    for(const issue of report.issues){const button=make("button",issue.title||issue.id||"定位问题");button.append(make("small",issue.detail||""));button.onclick=()=>fit([issue.geometry]);$("surface-issues").append(button);}
    $("surface-geometry-state").textContent=value.candidate_accepted?"工程研究候选（已接受） · 正式交付仍阻断":"本次从原件生成的预览候选（未接受） · 正式交付仍阻断";
    $("surface-binding").textContent=`候选 SHA256：${report.context.candidate_sha256||"未提供"}\n来源快照：${report.context.source_snapshot_id||"未提供"}\n来源内容：${report.context.source_content_hash||"未提供"}`;
    showFidelity(report.fidelity);fitAll();controls();
  }
  async function getGeometry(jobId=null){
    clearFidelity("正在读取本次候选的来源保真定位…");paint();
    const s=session,id=owner,g=epoch,bound=version(),ticket=++geometrySequence;
    const expectedId=jobId?s.compileJob?.result?.candidate?.candidate_id:project?.candidate?.candidate_id;
    if(!expectedId)return;
    try{const value=await api(`/projects/${id}/surface-rebuild/geometry`+(jobId?`?job_id=${encodeURIComponent(jobId)}`:""));
      if(!current(s,id,g)||ticket!==geometrySequence||bound!==version())return;
      if(!value?.available){clearGeometry();$("surface-geometry-state").textContent=value?.reason||"本次候选几何不可用";return;}
      if(!validReport(value,expectedId)){clearGeometry();$("surface-geometry-state").textContent="候选身份或局部坐标框架不匹配，未显示几何。";return;}
      showGeometry(value);
    }catch(error){if(current(s,id,g)&&ticket===geometrySequence&&bound===version()){clearGeometry();$("surface-geometry-state").textContent="候选几何读取失败："+error.message;}}
  }
  const gateNames={G8:"来源保真",G11:"结构与几何","G11-edge-contacts":"车道边缘接口","baseline-comparison":"完整基线比较","delivery-decision":"正式交付裁决","production-delivery":"正式交付","local-source-supported-surface":"辅助铺面来源支持","fresh-source-generation":"从原件生成","whole-map-validation":"整图检查","static-scoreboard":"静态草案评分"};
  const statusNames={PASS:"通过",FAIL:"未通过",BLOCKED:"阻断",UNAVAILABLE:"不可评估",COMPLETED:"已完成",REVIEW:"待复核",REVIEW_REQUIRED:"待复核"};
  const gateLabel=gate=>gateNames[gate]?`${gateNames[gate]}（${gate}）`:gate||"检查";
  const statusLabel=value=>value?(statusNames[value]?`${value}（${statusNames[value]}）`:String(value)):"未完成";
  function reasonLabel(reason){
    if(reason==="unavailable-original-generation-failed")return "原流程完整生成失败，不能进行完整基线比较。";
    if(typeof reason==="string")return reason;
    if(reason?.code==="crs_not_absolutely_verified")return "绝对坐标尚未经过独立核验；来源内部一致不代表绝对位置准确。";
    if(reason?.code==="required_gate_not_pass")return `${gateLabel(reason.gate_id)}未满足交付要求：${statusLabel(reason.gate_status)}。`;
    return reason?.message||reason?.reason||reason?.code||"未解决的阻断项";
  }
  function showChecks(validation,stale=false){
    $("surface-checks").replaceChildren();if(!validation)return;
    $("surface-checks").append(make("strong",(stale?"旧检查已过期 · ":"")+"交付裁决 "+statusLabel(validation.decision)));
    for(const check of validation.checks||[]){
      if(check.gate==="validation-byte-binding")continue;
      const text=check.tiers?"原策略评分："+Object.entries(check.tiers).map(([tier,row])=>tier+" "+statusLabel(row.status)).join(" · "):gateLabel(check.gate)+"："+statusLabel(check.status);
      $("surface-checks").append(make("div",text));
      if(check.reason)$("surface-checks").append(make("div",reasonLabel(check.reason)));
      for(const reason of check.blocked_reasons||[])$("surface-checks").append(make("div",reasonLabel(reason)));
    }
  }
  function showDescriptor(){
    const descriptor=session.descriptor;$("surface-actions").replaceChildren();
    for(const action of descriptor?.capability?.actions||[])$("surface-actions").append(make("li",String(action)));
    $("surface-reason").textContent=descriptor?.reason||(descriptor?.enabled?"固定来源操作已启用。每次预览和编译都从原始 SHP 重新生成。":"请先启用已核验的来源重建能力。");controls();
  }
  async function descriptor(){
    const s=session,id=owner,g=epoch,bound=version();s.descriptor=null;controls();
    try{const value=await api(`/projects/${id}/surface-rebuild`);if(!current(s,id,g)||bound!==version())return false;
      if(value.supported&&(!value.command_template||value.command_template.type!==TYPE||!Array.isArray(value.capability?.actions)||value.capability.actions.length!==3))throw new Error("服务器未提供完整的固定操作说明");
      s.descriptor=value;showDescriptor();return Boolean(value.supported);
    }catch(error){if(current(s,id,g)&&bound===version()){s.descriptor={supported:false,enabled:false,reason:error.message};showDescriptor();state("能力不可用："+error.message);}return false;}
  }
  async function work(fn){
    if(!session||session.mutation||busy)return;const s=session,id=owner,g=epoch,mark={};s.mutation=mark;controls();
    const valid=()=>current(s,id,g);
    try{await fn({s,id,g,valid});}catch(error){if(valid()){state(error.message);notify(error.message,true);}}
    finally{if(s.mutation===mark)s.mutation=null;if(valid())controls();}
  }
  function applyOwn(value){applying=true;try{applyProject(value);}finally{applying=false;}}
  function reconcile(){
    if(session?.ticket&&!ticketMatchesBase()&&!ticketMatchesDraft())session.ticket=null;
    clearGeometry();if(project?.validation)showChecks(project.validation,project.status.validation_stale);else if(candidateCurrent())showChecks({decision:"BLOCKED",checks:project.candidate.checks});else $("surface-checks").replaceChildren();controls();
  }
  async function open(){
    if(!project||busy)return;owner=project.project_id;session=sessions.get(owner)||freshSession();sessions.set(owner,session);epoch++;
    clearGeometry();$("surface-checks").replaceChildren();if(!panel.open)panel.showModal();state("正在读取此工程的来源重建能力…");controls();
    const s=session,id=owner,g=epoch,supported=await descriptor();if(!current(s,id,g))return;
    if(!supported){clearGeometry();$("surface-checks").replaceChildren();$("surface-actions").replaceChildren();state("来源重建不可用："+(s.descriptor?.reason||"此源工程暂无已登记的重建能力。"));controls();return;}reconcile();
    if(!s.descriptor.enabled){state(s.descriptor.reason||"启用后才能预览和保存来源重建草稿。");return;}
    if(running(s.compileJob))poll("compile",s.compileJob.job_id,s,id,g);
    else if(running(s.checkJob))poll("check",s.checkJob.job_id,s,id,g);
    else if(compiled()&&(ticketMatchesBase()||ticketMatchesDraft())){state("本次候选已生成，接受前须确认保存同一草稿。原 FAIL 和 BLOCKED 继续保留。");showChecks({decision:"BLOCKED",checks:s.compileJob.result.candidate.checks});await getGeometry(s.compileJob.job_id);}
    else {state(intents().length===1?"已载入确认的重建草稿；可编译或查看已接受候选。":"请计算预览，核对固定操作后确认并保存草稿。");if(candidateCurrent())await getGeometry();}
    if(project.validation)showChecks(project.validation,project.status.validation_stale);controls();
  }
  function close(){if(session?.mutation)return;epoch++;clearGeometry();if(panel.open)panel.close();refreshJobList();}
  function resumedStatus(job,kind){
    if(running(job))return kind==="check"?"整图检查运行中；原评分规则与阻断项保持。":"正在从原始 SHP 生成、重建并核对实际候选；关闭面板不会取消任务。";
    if(job.state==="succeeded")return kind==="check"?(job.result?.status==="VALIDATED"?"整图检查已完成；核对 FAIL/BLOCKED 后保存检查结果。":"检查被拒绝："+(job.result?.error?.message||"未完成")):(job.result?.status==="COMPILED"?"本次候选已生成；原 FAIL 和 BLOCKED 保留，尚未自动接受。":"重建被拒绝："+(job.result?.error?.message||"未通过检查")+"。已保存草稿与旧候选保留。");
    return ({cancelled:"任务已取消，未接受任何结果。",timed_out:"任务超时，未接受任何结果。",orphaned:"历史任务状态未确认，请明确重新编译；不会自动重启。",failed:"任务失败："+(job.error||"未知原因")})[job.state]||job.state;
  }
  async function poll(kind,jobId,s,id,g){
    const field=kind==="check"?"checkJob":"compileJob";if(!current(s,id,g)||s[field]?.job_id!==jobId)return;
    const observation=++s.pollSequence[kind],valid=()=>current(s,id,g)&&s[field]?.job_id===jobId&&s.pollSequence[kind]===observation;
    try{const job=await api(`/projects/${id}/jobs/${jobId}`);if(!valid())return;s[field]=job;
      state((job.historical?"历史只读记录 · ":"")+resumedStatus(job,kind)+(job.archive?.state==="failed"?" 归档失败，结果不能被接受或保存。":""));
      if(running(job))setTimeout(()=>{if(valid())poll(kind,jobId,s,id,g);},900);
      else if(kind==="check"&&job.result?.status==="VALIDATED")showChecks(job.result.validation,s.checkTarget!==version()||job.historical);
      else if(kind==="compile"&&compiled()&&(ticketMatchesBase()||ticketMatchesDraft())){showChecks({decision:"BLOCKED",checks:job.result.candidate.checks});await getGeometry(jobId);}
      controls();
    }catch(error){if(valid()){state("任务状态暂不可读取："+error.message+"；继续查询同一任务。");setTimeout(()=>{if(valid())poll(kind,jobId,s,id,g);},2000);}}
  }
  $("surface-enable").onclick=()=>work(async({s,id,valid})=>{const bound=version(),value=await api(`/projects/${id}/surface-rebuild/enable`,requestBody("enable",()=>({base_revision:project.revision,command_id:crypto.randomUUID()})));if(!valid()||bound!==version())return;delete s.requests.enable;applyOwn(value.project);await descriptor();if(valid())state("固定来源操作已启用。先计算预览，核对三项操作与原来源空区。");});
  $("surface-preview").onclick=()=>work(async({s,id,g,valid})=>{
    if(intents().length!==0||!s.descriptor?.enabled||anyRunning())return;
    const bound=version(),base={revision:project.revision,epoch:project.draft_epoch,content:project.content_hash,context:stable(project.context)};
    const body=requestBody("preview",()=>({base_revision:base.revision,command:{...clone(s.descriptor.command_template),command_id:crypto.randomUUID()},request_id:crypto.randomUUID()}));
    const value=await api(`/projects/${id}/surface-rebuild/preview`,body);
    if(!valid()||bound!==version())return;
    delete s.requests.preview;
    s.ticket={...value.preview,base_epoch:base.epoch,base_content:base.content,base_context:base.context};s.compileJob=value.job;s.checkJob=null;clearGeometry();
    state("已启动从原件生成的预览；草稿尚未确认保存。");poll("compile",value.job.job_id,s,id,g);
  });
  $("surface-save").onclick=()=>work(async({s,id,valid})=>{
    if(intents().length!==0||!ticketMatchesBase())return;const bound=version(),value=await api(`/projects/${id}/commands`,{base_revision:project.revision,command:clone(s.ticket.command)});
    if(!valid()||bound!==version())return;applyOwn(value);state("同一重建草稿已确认保存。候选仍需明确接受，正式交付保持阻断。");if(compiled()&&ticketMatchesDraft())await getGeometry(s.compileJob.job_id);
  });
  $("surface-compile").onclick=()=>work(async({s,id,g,valid})=>{
    if(intents().length!==1||anyRunning())return;const bound=version(),ticket={base_revision:project.revision,target_revision:project.revision,target_draft_epoch:project.draft_epoch,target_content_hash:project.content_hash,command:clone(intents()[0])};
    const value=await api(`/projects/${id}/surface-rebuild/compile`,requestBody("compile",()=>({base_revision:project.revision,request_id:crypto.randomUUID()})));if(!valid()||bound!==version())return;delete s.requests.compile;
    s.ticket=ticket;s.compileJob=value.job;s.checkJob=null;clearGeometry();state("正在从原件编译已保存草稿；旧候选保留。");poll("compile",value.job.job_id,s,id,g);
  });
  $("surface-accept").onclick=()=>work(async({s,id,valid})=>{if(!canAccept())return;const bound=version(),value=await api(`/projects/${id}/surface-rebuild/accept`,requestBody("accept",()=>({base_revision:project.revision,command_id:crypto.randomUUID(),job_id:s.compileJob.job_id}),s.compileJob.job_id));if(!valid()||bound!==version())return;delete s.requests.accept;applyOwn(value);s.ticket=null;state("已接受为工程研究候选；T1/T2 与正式交付结论以完整检查为准。接受不是发布通过。");await getGeometry();});
  $("surface-check").onclick=()=>work(async({s,id,g,valid})=>{if(!candidateCurrent()||anyRunning())return;const bound=version(),value=await api(`/projects/${id}/checking/start`,requestBody("check",()=>({base_revision:project.revision,request_id:crypto.randomUUID()})));if(!valid()||bound!==version())return;delete s.requests.check;s.checkJob=value.job;s.checkTarget=bound;state("正在检查已接受候选的实际字节；不会自动保存或开放交付。");poll("check",value.job.job_id,s,id,g);});
  $("surface-attach").onclick=()=>work(async({s,id,valid})=>{if(!canAttach())return;const bound=version(),value=await api(`/projects/${id}/checking/attach`,requestBody("attach",()=>({base_revision:project.revision,command_id:crypto.randomUUID(),job_id:s.checkJob.job_id}),s.checkJob.job_id));if(!valid()||bound!==version())return;delete s.requests.attach;applyOwn(value);showChecks(value.validation,value.status.validation_stale);state("完整检查已保存；FAIL 和 BLOCKED 随研究包保留，正式交付未开放。");await getGeometry();});
  $("surface-cancel").onclick=()=>work(async({s,id,g,valid})=>{const kind=running(s.checkJob)?"check":"compile",field=kind==="check"?"checkJob":"compileJob",job=s[field];if(!running(job))return;s.pollSequence[kind]++;const value=await api(`/projects/${id}/jobs/${job.job_id}/cancel`,{});if(!valid()||s[field]?.job_id!==job.job_id)return;s[field]=value;state("已请求取消；未接受任何新结果。");if(running(value))poll(kind,job.job_id,s,id,g);controls();});
  $("surface-export").onclick=()=>work(async({id,valid})=>{
    if(!candidateCurrent()||!project.validation||project.status.validation_stale)return;const bound=version();state("正在核对当前候选和已保存检查，生成研究包…");
    const receipt=await api(`/projects/${id}/research-exports`,{base_revision:project.revision});if(!valid()||bound!==version())return;
    const response=await fetch(`/api/projects/${id}/research-exports/${encodeURIComponent(receipt.export_id)}`,{headers:{Authorization:"Bearer "+token}});
    if(!response.ok){const value=await response.json();throw new Error(value.detail||"研究包读取失败");}
    const data=await response.arrayBuffer(),hash=Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256",data)),value=>value.toString(16).padStart(2,"0")).join("");
    if(!valid()||bound!==version())return;
    if(hash!==receipt.sha256||data.byteLength!==receipt.size_bytes||response.headers.get("X-Mapforge-Purpose")!=="RESEARCH_ONLY")throw new Error("研究包字节校验失败，未下载。");
    const url=URL.createObjectURL(new Blob([data],{type:"application/zip"})),link=make("a");link.href=url;link.download=receipt.file_name;document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),60000);
    state("研究包已核验并发起下载；完整 FAIL/BLOCKED 随包保留。这不是正式交付包。");
  });
  function changed(reset){
    geometrySequence++;buttons();
    if(owner!==project?.project_id||reset){epoch++;clearGeometry();if(panel.open)panel.close();owner=null;session=null;$("surface-checks").replaceChildren();controls();paint();return;}
    if(!session)return;reconcile();
    if(!applying){epoch++;session.descriptor=null;state("工程已变化，原预览及检查需核对。请重新打开来源重建面板读取当前状态。");controls();}
  }
  $("open-surface-editing").onclick=open;$("surface-close").onclick=close;$("surface-fit").onclick=fitAll;
  $("surface-fidelity-clear").onclick=()=>{if(session)session.selectedFidelity=null;$("surface-fidelity-selection").textContent="未选中比较车道";$("surface-fidelity-primary").textContent="";$("surface-fidelity-refs").replaceChildren();$("surface-fidelity-clear").disabled=true;fidelityButtons();fitAll();};
  function zoom(factor){if(!session?.geometry)return;view.scale=Math.max(1e-8,Math.min(1e8,view.scale*factor));paint();}
  $("surface-zoom-in").onclick=()=>zoom(1.5);$("surface-zoom-out").onclick=()=>zoom(1/1.5);
  panel.addEventListener("cancel",event=>{event.preventDefault();close();});panel.addEventListener("close",()=>{if(panel.open)return;epoch++;geometrySequence++;drag=null;clearFidelity();paint();});
  plot.onpointerdown=event=>{if(!session?.geometry)return;drag={x:event.offsetX,y:event.offsetY};plot.setPointerCapture(event.pointerId);};
  plot.onpointermove=event=>{if(!session?.geometry)return;const p=world(event.offsetX,event.offsetY);$("surface-coordinates").textContent=`局部 x ${p[0].toFixed(3)} m · y ${p[1].toFixed(3)} m · 绝对位置未核验`;if(drag){view.x-=(event.offsetX-drag.x)/view.scale;view.y+=(event.offsetY-drag.y)/view.scale;drag={x:event.offsetX,y:event.offsetY};paint();}};
  plot.onpointerup=plot.onpointercancel=plot.onlostpointercapture=()=>{drag=null;};plot.addEventListener("wheel",event=>{if(!session?.geometry)return;event.preventDefault();const a=world(event.offsetX,event.offsetY);zoom(Math.exp(-Math.max(-150,Math.min(150,event.deltaY))*.002));const b=world(event.offsetX,event.offsetY);view.x+=a[0]-b[0];view.y+=a[1]-b[1];paint();},{passive:false});
  new ResizeObserver(paint).observe(plot);buttons();controls();return {buttons,changed};
})();
