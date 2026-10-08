"use strict";
// This pane draws sampled, hash-checked XODR output from the server. Numeric
// input never deforms a displayed candidate or silently accepts a draft.
(() => {
  const panel = $("editing-dialog"), plot = $("edit-map"), pen = plot.getContext("2d");
  let descriptor=null, geometry=null, ticket=null, editJob=null, checkJob=null, editBusy=false;
  let owner=null, epoch=0, editView={x:0,y:0,scale:1}, drag=null, inputDirty=false;
  let viewSequence=0;
  const sessions=new Map();
  const state = text => { $("edit-status").textContent=text; };
  const delta = () => { const raw=$("normal-delta").value; return raw.trim()===""?NaN:Number(raw)/1000; };
  const savedDelta = () => { const intents=project?.intents||[];for(let i=intents.length-1;i>=0;i--)if(intents[i].type==="shared_boundary_c2_normal_delta")return intents[i].parameters.normal_delta_m;return 0; };
  const validDelta = () => Number.isFinite(delta()) && Math.abs(delta())<=0.1;
  const running = () => editJob?.state==="running"||checkJob?.state==="running";
  const matchesInput = () => ticket && ticket.command.parameters.normal_delta_m===delta();
  const matchesDraft = () => ticket && ticket.target_content_hash===project?.content_hash && ticket.target_revision===project?.revision && ticket.target_draft_epoch===project?.draft_epoch;
  function controls() {
    const locked=editBusy||!project||project.status?.read_only||!descriptor?.enabled;
    $("normal-delta").disabled=locked;
    $("enable-editing").hidden=!descriptor?.supported||descriptor?.enabled;
    $("enable-editing").disabled=editBusy||project?.status?.read_only;
    $("preview-edit").disabled=locked||running()||!validDelta();
    $("save-edit").disabled=locked||!validDelta()||(!inputDirty&&delta()===savedDelta());
    $("compile-edit").disabled=locked||running()||inputDirty;
    $("accept-edit").disabled=locked||editJob?.state!=="succeeded"||editJob?.result?.status!=="COMPILED"||!matchesDraft()||!matchesInput();
    $("validate-edit").disabled=locked||running()||inputDirty||!project?.candidate||project.status.candidate_stale;
    $("attach-validation").disabled=locked||checkJob?.state!=="succeeded"||checkJob?.result?.status!=="VALIDATED"||checkJob.base_revision!==project?.revision||checkJob.content_hash!==project?.content_hash||checkJob.historical;
    $("cancel-edit-job").hidden=!running();
    $("cancel-edit-job").disabled=editBusy;
    $("close-editing").disabled=editBusy;
  }
  async function work(fn) {
    if(editBusy)return;editBusy=true;controls();
    try{await fn();}catch(e){state(e.message);notify(e.message,true);}finally{editBusy=false;controls();}
  }
  function command() {
    const cap=descriptor.capability;
    return {command_id:crypto.randomUUID(),type:cap.intent_type,source_ref:cap.source_ref,
      scope:cap.scope,parameters:{normal_delta_m:delta(),baseline_sha256:cap.baseline_sha256||cap.coordinate_evidence.baseline_sha256}};
  }
  function lines() {
    if(!geometry)return [];
    return [...(geometry.baseline?.lines||[]).map(x=>({...x,color:"#8092a2",width:1.5})),
      ...(geometry.source?.lines||[]).map(x=>({...x,color:"#55c8c0",width:1.6})),
      ...(geometry.candidate?.lines||[]).map(x=>({...x,color:geometry.candidate_stale?"#a68a5c":"#ffd173",width:2}))];
  }
  function screen(p){return [(p[0]-editView.x)*editView.scale+plot.clientWidth/2,plot.clientHeight/2-(p[1]-editView.y)*editView.scale];}
  function world(x,y){return [(x-plot.clientWidth/2)/editView.scale+editView.x,(plot.clientHeight/2-y)/editView.scale+editView.y];}
  function paint() {
    const r=devicePixelRatio||1,w=plot.clientWidth,h=plot.clientHeight;
    if(plot.width!==Math.round(w*r)||plot.height!==Math.round(h*r)){plot.width=Math.round(w*r);plot.height=Math.round(h*r);}
    pen.setTransform(r,0,0,r,0,0);pen.clearRect(0,0,w,h);
    pen.strokeStyle="#233346";pen.lineWidth=1;pen.beginPath();for(let x=0;x<w;x+=50){pen.moveTo(x,0);pen.lineTo(x,h);}for(let y=0;y<h;y+=50){pen.moveTo(0,y);pen.lineTo(w,y);}pen.stroke();
    for(const line of lines()){pen.strokeStyle=line.color;pen.lineWidth=line.width;pen.beginPath();for(let i=0;i<line.points.length;i++){const p=screen(line.points[i]);if(i===0)pen.moveTo(...p);else pen.lineTo(...p);}pen.stroke();}
    if(geometry){const metres=100/editView.scale;pen.strokeStyle="#a9bccf";pen.lineWidth=2;pen.beginPath();pen.moveTo(20,h-30);pen.lineTo(120,h-30);pen.stroke();pen.fillStyle="#a9bccf";pen.font="12px sans-serif";pen.fillText(`${metres.toPrecision(3)} m`,20,h-40);}
  }
  function fitEdit() {
    const points=lines().flatMap(x=>x.points);if(!points.length){paint();return;}
    let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;
    for(const p of points){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}
    editView={x:(x0+x1)/2,y:(y0+y1)/2,scale:Math.min(Math.max(100,plot.clientWidth-90)/Math.max(1,x1-x0),Math.max(100,plot.clientHeight-90)/Math.max(1,y1-y0))};paint();
  }
  async function getGeometry(jobId, fit=false) {
    const g=epoch,id=owner,sequence=++viewSequence,valueAtStart=delta(),revisionAtStart=project.revision;
    const value=await api(`/projects/${id}/editing/geometry`+(jobId?`?job_id=${encodeURIComponent(jobId)}`:""));
    if(g!==epoch||id!==owner||sequence!==viewSequence||valueAtStart!==delta()||revisionAtStart!==project.revision)return;
    geometry=value;if(fit)fitEdit();else paint();
  }
  async function openEditor() {
    if(!project)return;
    if(owner===project.project_id&&descriptor){
      epoch++;panel.showModal();controls();paint();
      if(editJob?.state==="running")pollEdit(owner,editJob.job_id,epoch);
      if(checkJob?.state==="running")pollCheck(owner,checkJob.job_id,epoch);
      if(!inputDirty){$("normal-delta").value=String(savedDelta()*1000);controls();}
      if(!ticket&&descriptor.enabled)await work(()=>getGeometry(null));
      if(project.validation)showValidation(project.validation,project.status.validation_stale);
      if(project.status.candidate_stale)state("当前草稿与旧候选不同；旧结果仅作对照，需重新编译并接受。");
      return;
    }
    owner=project.project_id;epoch++;descriptor=null;geometry=null;ticket=null;editJob=null;checkJob=null;inputDirty=false;
    $("normal-delta").value=String(savedDelta()*1000);$("edit-checks").textContent="";
    state("正在读取此工程的编辑能力…");panel.showModal();paint();controls();
    const id=owner,g=epoch;
    await work(async()=>{
      const value=await api(`/projects/${id}/editing`);if(g!==epoch)return;descriptor=value;
      $("edit-reason").textContent=value.reason||(value.enabled?"此处能力已登记。可输入连续位移，每个候选都要重新检查。":"请先启用已核验的编辑范围。");
      $("edit-scope").textContent=value.supported?"凤阁路—金剑路 · road12 · 车道 −1/−2 · s 76.269–110 m":"此源工程暂无已核验的几何编辑能力";
      state(value.enabled?(project.status.candidate_stale?"已载入草稿；旧候选已过期，仅作对照。请编译当前草稿。":"已载入当前草稿。输入目标后计算预览，或编译已保存草稿。"):value.reason||"启用编辑后显示实际基线。");
      if(value.enabled)await getGeometry(null,true);
      if(project.validation)showValidation(project.validation,project.status.validation_stale);
    });
  }
  $("open-editing").onclick=openEditor;
  function closeEditor() {
    if(editBusy)return;
    if(inputDirty&&!confirm("输入的位移尚未保存，是否放弃输入并返回？"))return;
    // Keep the owned task and its exact preview ticket across panel navigation.
    // Closing a pane is not cancellation or candidate acceptance.
    if(inputDirty){ticket=null;if(geometry)geometry={...geometry,candidate:null};state("未保存的位移输入已放弃；已保存草稿保持。");}
    inputDirty=false;$("normal-delta").value=String(savedDelta()*1000);epoch++;panel.close();refreshJobList();
  }
  $("close-editing").onclick=closeEditor;
  panel.addEventListener("cancel",e=>{e.preventDefault();closeEditor();});
  $("enable-editing").onclick=()=>work(async()=>{
    const result=await api(`/projects/${owner}/editing/enable`,{base_revision:project.revision,command_id:crypto.randomUUID()});
    applyProject(result.project);descriptor={supported:true,enabled:true,capability:result.capability};
    $("edit-reason").textContent="已登记此边界范围。每个连续目标仍须通过真实局部与来源检查。";
    await getGeometry(null,true);state("编辑已启用。输入法向位移后计算预览。");
  });
  $("normal-delta").oninput=()=>{
    viewSequence++;
    inputDirty=delta()!==savedDelta();
    if(geometry?.candidate){geometry={...geometry,candidate:null};paint();}
    state(validDelta()?"位移输入尚未编译；画布显示原基线与源边界。":"请输入 −100 至 100 mm 内的有限数值。");controls();
  };
  $("preview-edit").onclick=()=>work(async()=>{
    const cmd=command(),base=project.revision;
    const response=await api(`/projects/${owner}/editing/preview`,{base_revision:base,command:cmd,request_id:crypto.randomUUID()});
    ticket=response.preview;editJob=response.job;state("正在计算真实候选；草稿尚未确认保存。");
    pollEdit(owner,editJob.job_id,epoch);
  });
  $("save-edit").onclick=()=>work(async()=>{
    const reuse=matchesInput()&&ticket.base_revision===project.revision;
    const cmd=reuse?ticket.command:command();
    const result=await api(`/projects/${owner}/commands`,{base_revision:project.revision,command:cmd});
    applyProject(result);inputDirty=false;
    if(!reuse){ticket=null;geometry=geometry?{...geometry,candidate:null}:null;paint();}
    else if(editJob?.result?.status==="COMPILED")await getGeometry(editJob.job_id);
    state("草稿已保存。预览被拒绝也可保留草稿；只有有效且匹配的候选才能被接受。");
  });
  $("compile-edit").onclick=()=>work(async()=>{
    const response=await api(`/projects/${owner}/editing/compile`,{base_revision:project.revision,request_id:crypto.randomUUID()});
    ticket={base_revision:project.revision,target_revision:project.revision,target_draft_epoch:project.draft_epoch,target_content_hash:project.content_hash,command:{parameters:{normal_delta_m:savedDelta()}}};
    editJob=response.job;state("正在编译已保存草稿…");pollEdit(owner,editJob.job_id,epoch);
  });
  async function pollEdit(id,jobId,g) {
    if(g!==epoch||id!==owner||jobId!==editJob?.job_id)return;
    try {
      const job=await api(`/projects/${id}/jobs/${jobId}`);if(g!==epoch||id!==owner||jobId!==editJob?.job_id)return;editJob=job;
      if(job.state==="running"){state("正在生成真实 XODR 并核对来源残差与局部约束…");setTimeout(()=>pollEdit(id,jobId,g),750);}
      else if(job.state==="succeeded") {
        if(job.result?.status==="COMPILED") {
          state(matchesInput()?"候选已生成，局部检查通过。确认草稿后可接受此候选；整图检查尚未运行。":"较早输入的候选已生成；当前输入需重新预览。");
          $("edit-checks").textContent=`实际候选 SHA256：${job.result.evidence?.candidate_sha256||"未知"}。局部检查完成不代表正式交付通过。`;
          if(matchesInput())await getGeometry(jobId);
        } else {state("候选被拒绝："+(job.result?.error?.message||"检查未通过")+"。草稿仍可保存，旧候选保留。");$("edit-checks").textContent="该输入没有可接受的候选。";}
      } else {state(({cancelled:"计算已取消，未接受任何结果。",timed_out:"计算超时，草稿及旧候选保留。",orphaned:"历史任务状态未确认，请重新编译当前草稿。",failed:"工作进程失败："+(job.error||"未知原因")})[job.state]||job.state);}
      if(job.archive?.state==="failed")state($("edit-status").textContent+" 任务结果未持久化，不能接受候选。");
      controls();
    } catch(e) {if(g===epoch&&jobId===editJob?.job_id){state("任务状态暂不可读取："+e.message+"；继续查询同一任务。");setTimeout(()=>pollEdit(id,jobId,g),2000);}}
  }
  $("accept-edit").onclick=()=>work(async()=>{
    const result=await api(`/projects/${owner}/editing/accept`,{base_revision:project.revision,command_id:crypto.randomUUID(),job_id:editJob.job_id});
    applyProject(result);ticket=null;inputDirty=false;await getGeometry(null);
    state("已接受与当前草稿绑定的真实候选并保存。正式交付仍需完整检查。");
  });
  $("cancel-edit-job").onclick=()=>work(async()=>{if(checkJob?.state==="running")checkJob=await api(`/projects/${owner}/jobs/${checkJob.job_id}/cancel`,{});else if(editJob?.state==="running")editJob=await api(`/projects/${owner}/jobs/${editJob.job_id}/cancel`,{});state("已请求取消；任务结果不会自动被接受。");});
  function showValidation(validation,stale=false) {
    $("edit-checks").replaceChildren();
    const labels={BLOCKED:"阻断交付",REVIEW:"待复核",DELIVERABLE:"满足当前裁决规则"};
    const gates={G8:"来源保真（G8）",G11:"结构与几何（G11）","G11-edge-contacts":"车道边缘接口","delivery-decision":"正式交付裁决"};
    const statuses={PASS:"通过",FAIL:"未通过",BLOCKED:"阻断",REVIEW_REQUIRED:"待复核",UNAVAILABLE:"未完成",COMPLETED:"已完成"};
    $("edit-checks").append(make("strong",(stale?"旧检查已过期 · ":"")+(labels[validation.decision]||validation.decision)));
    for(const check of validation.checks){
      if(check.gate==="validation-byte-binding")continue;
      let text=(gates[check.gate]||check.gate)+"："+(statuses[check.status]||check.status);
      if(check.gate==="static-scoreboard")text="草案评分："+Object.entries(check.tiers||{}).map(([k,v])=>k+" "+(statuses[v.status]||v.status)).join(" · ");
      $("edit-checks").append(make("div",text));
      if(check.gate==="delivery-decision")for(const reason of (check.blocked_reasons||[])){
        const text=typeof reason==="string"?reason:reason.code==="crs_not_absolutely_verified"?"缺少独立绝对坐标核验材料。":reason.code==="required_gate_not_pass"?`${gates[reason.gate_id]||reason.gate_id}未通过。`:reason.message||reason.reason||"待处理的阻断项："+(reason.code||"未分类");
        $("edit-checks").append(make("div",text));
      }
    }
  }
  $("validate-edit").onclick=()=>work(async()=>{
    const response=await api(`/projects/${owner}/checking/start`,{base_revision:project.revision,request_id:crypto.randomUUID()});
    checkJob=response.job;state("正在对已接受候选做整图检查，与固定基线使用同一评分策略…");pollCheck(owner,checkJob.job_id,epoch);
  });
  async function pollCheck(id,jobId,g) {
    if(g!==epoch||id!==owner||jobId!==checkJob?.job_id)return;
    try {
      const job=await api(`/projects/${id}/jobs/${jobId}`);if(g!==epoch||id!==owner||jobId!==checkJob?.job_id)return;checkJob=job;
      if(job.state==="running"){state("整图检查运行中：实际 XODR、来源保真、静态门禁和交付裁决。");setTimeout(()=>pollCheck(id,jobId,g),1000);}
      else if(job.state==="succeeded"&&job.result?.status==="VALIDATED"){
        showValidation(job.result.validation,job.stale);state(job.stale?"检查对应旧工程版本，不能保存为当前结果。":"整图检查已完成。请核对裁决，并保存与当前候选绑定的结果。");
      } else {state(job.result?.error?.message||job.error||({cancelled:"整图检查已取消。",timed_out:"整图检查超时。",orphaned:"历史检查状态未知，请重新检查。"})[job.state]||"检查未完成。");}
      if(job.archive?.state==="failed")state($("edit-status").textContent+" 检查归档失败，结果不能被保存。");
      controls();
    }catch(e){if(g===epoch&&jobId===checkJob?.job_id){state("检查任务暂不可读取："+e.message);setTimeout(()=>pollCheck(id,jobId,g),2000);}}
  }
  $("attach-validation").onclick=()=>work(async()=>{
    const value=await api(`/projects/${owner}/checking/attach`,{base_revision:project.revision,command_id:crypto.randomUUID(),job_id:checkJob.job_id});
    applyProject(value);showValidation(value.validation,value.status.validation_stale);state("整图检查结果已保存。交付裁决以实际阻断项为准。");
  });
  $("edit-fit").onclick=fitEdit;$("edit-zoom-in").onclick=()=>{editView.scale*=1.5;paint();};$("edit-zoom-out").onclick=()=>{editView.scale/=1.5;paint();};
  plot.onpointerdown=e=>{drag={x:e.offsetX,y:e.offsetY};plot.setPointerCapture(e.pointerId);};
  plot.onpointermove=e=>{const p=world(e.offsetX,e.offsetY);$("edit-coordinates").textContent=`局部 x ${p[0].toFixed(3)} m · y ${p[1].toFixed(3)} m · 绝对坐标未核验`;if(drag){editView.x-=(e.offsetX-drag.x)/editView.scale;editView.y+=(e.offsetY-drag.y)/editView.scale;drag={x:e.offsetX,y:e.offsetY};paint();}};
  plot.onpointerup=plot.onpointercancel=()=>{drag=null;};
  plot.addEventListener("wheel",e=>{e.preventDefault();const a=world(e.offsetX,e.offsetY);editView.scale*=Math.exp(-Math.max(-150,Math.min(150,e.deltaY))*.002);const b=world(e.offsetX,e.offsetY);editView.x+=a[0]-b[0];editView.y+=a[1]-b[1];paint();},{passive:false});
  new ResizeObserver(paint).observe(plot);
  window.addEventListener("beforeunload",e=>{if(inputDirty){e.preventDefault();e.returnValue="";}});
  window.Editor={changed(reset){
    viewSequence++;
    if(owner!==project?.project_id){
      if(owner)sessions.set(owner,{descriptor,geometry,ticket,editJob,checkJob});
      epoch++;owner=project?.project_id;
      ({descriptor=null,geometry=null,ticket=null,editJob=null,checkJob=null}=sessions.get(owner)||{});
      inputDirty=false;if(panel.open)panel.close();
    } else if(reset){epoch++;inputDirty=false;if(panel.open)panel.close();}
    const previewBase=ticket&&ticket.base_revision===project?.revision;
    if(ticket&&!matchesDraft()&&!previewBase){ticket=null;if(geometry)geometry={...geometry,candidate:null};}
    if(!ticket&&geometry?.candidate)geometry={...geometry,candidate_stale:Boolean(project?.status?.candidate_stale)};
    if(checkJob?.result?.status==="VALIDATED")showValidation(checkJob.result.validation,checkJob.base_revision!==project?.revision||checkJob.content_hash!==project?.content_hash);
    if(project?.validation)showValidation(project.validation,project.status.validation_stale);
    if(!inputDirty)$("normal-delta").value=String(savedDelta()*1000);
    controls();paint();
  }};
})();
