"use strict";
// Saved-project packages: export, chunked upload, background import and an
// explicit open. The page never opens, compiles or accepts an import itself.
window.Transfer=(()=>{
  let limits=null,running=false,imported=null;
  const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
  const size=n=>n<1048576?(n/1024).toFixed(1)+" KB":(n/1048576).toFixed(1)+" MB";
  function say(id,text,error=false){$(id).textContent=text;$(id).classList.toggle("error",error);}
  function explain(error){return error?String(error.message||"未记录原因").replace(/^(TransferRejected|RelocationRejected): /,"")+(error.code?`（${error.code}）`:""):"未记录原因";}
  function staleText(r){const old=[r.candidate_stale&&"候选",r.validation_stale&&"检查"].filter(Boolean);return old.length?`旧${old.join("和")}已过期`:"原工程没有已接受的候选或检查";}
  function buttons(){
    $("open-transfer").disabled=!limits;
    $("transfer-export").disabled=!limits||running||busy||!project;
    $("transfer-file").disabled=running;
    $("transfer-import").disabled=!limits||running||!$("transfer-file").files?.length;
    $("transfer-open").disabled=running||busy||!imported;
    $("transfer-export-scope").textContent=project?`当前工程：${project.name} · 已保存修订 ${project.revision}`:"尚未打开工程";
  }
  async function run(target,prefix,action){
    if(running)return;running=true;buttons();
    try{await action();}catch(e){say(target,prefix+e.message,true);if(!$("transfer-dialog").open)notify("工程迁移"+prefix+e.message,true);}
    finally{running=false;$("transfer-progress").hidden=true;buttons();}
  }
  async function wait(job){
    // The worker continues server-side; a few unreadable polls are not a failure.
    let misses=0;
    while(job.state==="running"){
      await sleep(500);
      try{job=await api("/transfers/jobs/"+encodeURIComponent(job.job_id));misses=0;}
      catch(e){if(++misses>=20)throw new Error("任务状态暂不可读取："+e.message+"；后台任务可能仍在进行，请稍后在本窗口重试");}
    }
    return job;
  }
  async function put(id,offset,chunk){
    const response=await fetch(`/api/transfers/uploads/${encodeURIComponent(id)}/chunks?offset=${offset}`,{method:"PUT",headers:{Authorization:"Bearer "+token,"X-Mapforge-CSRF":token,"Content-Type":"application/octet-stream"},body:chunk});
    const result=await response.json();if(!response.ok)throw new Error(typeof result.detail==="string"?result.detail:JSON.stringify(result.detail));return result;
  }
  function save(data,name){const url=URL.createObjectURL(data),link=make("a");link.href=url;link.download=name;document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),60000);}
  $("open-transfer").onclick=()=>{buttons();$("transfer-dialog").showModal();};
  $("transfer-close").onclick=()=>$("transfer-dialog").close();
  $("transfer-export").onclick=()=>run("transfer-export-status","未导出：",async()=>{
    if(pendingNote&&!confirm("未确认的待办输入不会进入工程包。继续导出已保存的工程？"))return;
    const id=project.project_id,revision=project.revision,name=project.name;
    say("transfer-export-status",`正在打包“${name}”已保存修订 ${revision}…`);
    const job=await wait(await api("/transfers/exports",{project_id:id,base_revision:revision,request_id:crypto.randomUUID()}));
    if(job.state!=="succeeded")throw new Error(explain(job.error));
    // The server rechecks project, source and history before serving bytes.
    const response=await fetch(`/api/transfers/jobs/${encodeURIComponent(job.job_id)}/package`,{headers:{Authorization:"Bearer "+token}});
    if(!response.ok){const error=await response.json();throw new Error(error.detail||"工程包读取失败");}
    const data=await response.blob(),sha=job.result.sha256;
    if(data.size!==job.result.size||response.headers.get("X-Content-SHA256")!==sha||await window.PackageHash.blob(data)!==sha)throw new Error("下载字节校验失败，未保存工程包。");
    save(data,job.result.filename);
    say("transfer-export-status",`已导出“${name}”修订 ${revision} · ${size(data.size)} · SHA256 ${sha.slice(0,12)}…。包内不含原始 SHP；导入方需登记同源原件，导入后须重新生成并检查。`);
  });
  $("transfer-file").onchange=()=>{const file=$("transfer-file").files?.[0];imported=null;$("transfer-result").hidden=true;say("transfer-import-status",file?`已选择 ${file.name} · ${size(file.size)}`:"尚未选择工程包。");buttons();};
  $("transfer-import").onclick=()=>run("transfer-import-status","未导入：",async()=>{
    const file=$("transfer-file").files[0];
    if(!file.name.endsWith(limits.file_suffix))throw new Error(`请选择工作台导出的 *${limits.file_suffix} 文件；下载时若被改名，请恢复该后缀。`);
    if(!file.size||file.size>limits.max_package_bytes)throw new Error(`工程包大小须在 1 字节到 ${size(limits.max_package_bytes)} 之间。`);
    imported=null;$("transfer-result").hidden=true;
    say("transfer-import-status","正在计算 SHA-256…");
    const total=file.size,digest=await window.PackageHash.blob(file,(done,all)=>say("transfer-import-status",`正在校验 ${size(done)} / ${size(all)}…`));
    let upload=await api("/transfers/uploads",{name:file.name,size:total,sha256:digest});
    $("transfer-progress").value=0;$("transfer-progress").hidden=false;
    for(let offset=0;offset<total;offset+=limits.chunk_bytes){
      upload=await put(upload.upload_id,offset,file.slice(offset,offset+limits.chunk_bytes));
      if(upload.state==="failed")throw new Error("上传校验失败："+explain(upload.error));
      $("transfer-progress").value=upload.received_size/total;
      say("transfer-import-status",`正在上传 ${size(upload.received_size)} / ${size(total)}…`);
    }
    if(upload.state!=="sealed")throw new Error("上传未完成封存，未开始导入。");
    say("transfer-import-status","上传完成，正在后台核对原件身份并导入；可关闭本窗口继续查看当前工程。");
    const job=await wait(await api("/transfers/imports",{upload_id:upload.upload_id,request_id:crypto.randomUUID()}));
    if(job.state!=="succeeded")throw new Error(explain(job.error)+"。工作区未新增工程。");
    imported=job.result;
    try{await refreshProjects();}catch(e){/* listing refresh is cosmetic; the explicit open below reads the store */}
    $("transfer-result-text").textContent=`已导入工程 ${imported.project_id} · 修订 ${imported.revision}。草稿与历史已保留；${staleText(imported)}，编辑能力需重新登记，候选需重新生成、独立检查并保存。正式交付仍未开放。`;
    $("transfer-result").hidden=false;
    say("transfer-import-status","导入完成。工程尚未打开，当前工程和草稿未改变。");
    if(!$("transfer-dialog").open)notify("工程包已导入，可在“工程迁移”中打开；当前工程未改变。");
  });
  $("transfer-open").onclick=()=>guarded(async()=>{
    if(!imported)return;const opened=imported,id=opened.project_id;
    await openProject(id);
    if(project?.project_id!==id)return;
    $("projects").value=id;$("transfer-dialog").close();
    notify(`已打开导入的工程。${staleText(opened)}；需重新生成、独立检查并保存后才能导出研究包，正式交付仍未开放。`);
  });
  api("/transfers").then(result=>{limits=result;buttons();}).catch(e=>say("transfer-import-status","迁移功能暂不可用："+e.message,true));
  return {buttons};
})();
