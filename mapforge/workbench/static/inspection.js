"use strict";
// Inspection results stay outside project intents and never grant delivery.
window.Inspection=(()=>{
  let mode=false, points=[], frame=null, sequence=0, frameSequence=0, owner=null, measuring=false;
  function current(id,g,ticket){return project?.project_id===id&&generation===g&&sequence===ticket;}
  function buttons(){
    $("measure-points").disabled=!project||busy||!frame;
    $("measure-points").setAttribute("aria-pressed",String(mode));
    $("measure-points").textContent=mode?"结束量距":"两点量距";
    const obj=byId.get(selected);
    $("measure-object").disabled=!project||busy||!frame||measuring||!obj||![3,5,13,15,23,25].includes(obj.shape_type)||pointsOf(obj).length<2;
    $("clear-measure").disabled=!project;
    const s=project?.status;
    $("research-export").disabled=!project||busy||s?.read_only||!project.candidate||!project.validation||s.candidate_stale||s.validation_stale;
  }
  function basis(value){$("measurement-basis").textContent=[value.display_label,value.reason,...(value.assumptions||[]),"绝对 CRS 未核验；量距不授予米制编辑或正式交付权限。"].filter(Boolean).join(" ");}
  function clear(){sequence++;mode=false;points=[];measuring=false;$("measurement-result").textContent=frame?frame.display_label+"；可测量原线或点击两点。":"正在读取量距口径…";buttons();draw();}
  async function changed(reset){
    buttons();
    if(!project)return;
    $("export-status").textContent=project.validation?(project.status.validation_stale?"旧检查已过期，请重新编译并检查当前草稿。":"当前检查裁决："+project.validation.decision+"；研究包保留全部阻断项。") :"接受候选并保存完整检查后可导出研究包。";
    if(!reset&&owner===project.project_id)return;
    owner=project.project_id;frame=null;clear();$("pick-panel").hidden=true;
    const id=owner,g=generation,ticket=++frameSequence;
    const frameCurrent=()=>project?.project_id===id&&generation===g&&frameSequence===ticket;
    try{const result=await api(`/projects/${id}/measurement`);if(!frameCurrent())return;frame=result;basis(result);$("measurement-result").textContent=result.display_label+"；不计高程。";buttons();}
    catch(e){if(frameCurrent()){$("measurement-result").textContent="量距口径不可用："+e.message;buttons();}}
  }
  async function measure(body){
    const id=project.project_id,g=generation,ticket=++sequence,objectId=body.object_id;
    measuring=true;buttons();$("measurement-result").textContent="计算原始点列长度…";
    try{const result=await api(`/projects/${id}/measurement`,body);if(!current(id,g,ticket)||(objectId&&selected!==objectId))return;
      const digits=result.unit==="m"?3:9;
      $("measurement-result").textContent=`${result.display_label}：${result.length.toFixed(digits)} ${result.unit} · ${result.point_count} 个原始点 · 不计高程`;
      basis(result.frame);
    }catch(e){if(current(id,g,ticket)&&(!objectId||selected===objectId))$("measurement-result").textContent="量距未完成："+e.message;}
    finally{if(current(id,g,ticket)){measuring=false;buttons();}}
  }
  function point(p){if(!mode)return false;if(points.length===2)points=[];points.push(p);draw();if(points.length===2)measure({points:points.map(p=>[...p])});else{sequence++;measuring=false;$("measurement-result").textContent="已取第一点，请在画布点击第二点。";buttons();}return true;}
  function overlay(){if(!points.length)return;ctx.save();ctx.strokeStyle="#ffcb73";ctx.fillStyle="#ffcb73";ctx.lineWidth=2;ctx.setLineDash([6,4]);ctx.beginPath();points.map(toScreen).forEach((p,i)=>{i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.stroke();ctx.setLineDash([]);for(const p of points.map(toScreen)){ctx.beginPath();ctx.arc(...p,4,0,Math.PI*2);ctx.fill();}ctx.restore();}
  function pick(hits){
    $("pick-candidates").replaceChildren();$("pick-panel").hidden=hits.length<2;
    if(hits.length===1){selectObject(hits[0].id);return;}
    for(const hit of hits){const obj=byId.get(hit.id),ref=obj.source_ref||{};
      const button=make("button",`${roleNames[roleOf(obj)]||roleOf(obj)} · ${obj.business_id||"无业务 ID"}`);
      button.append(make("small",`${ref.layer||"未知图层"} · 原记录 ${ref.record_index??"?"} · part ${ref.part_index??"空"}`));
      button.onclick=()=>{selectObject(hit.id);if(selected===hit.id)$("pick-panel").hidden=true;};$("pick-candidates").append(button);
    }
    if(hits.length>1)notify(`此处有 ${hits.length} 个源对象，请在右侧按原记录选择。`);
  }
  function escape(){const active=mode||!$("pick-panel").hidden;if(!active)return false;mode=false;$("pick-panel").hidden=true;buttons();return true;}
  $("measure-points").onclick=()=>{if(mode){mode=false;buttons();return;}clear();mode=true;buttons();$("measurement-result").textContent=frame.display_label+"；请在画布点击第一点。";};
  $("clear-measure").onclick=clear;
  $("measure-object").onclick=()=>{mode=false;points=[];draw();measure({object_id:selected});};
  $("close-picks").onclick=()=>$("pick-panel").hidden=true;
  $("research-export").onclick=()=>guarded(async()=>{
    const id=project.project_id,g=generation,revision=project.revision;
    $("export-status").textContent="正在核对产物并打包…";
    try{
      const receipt=await api(`/projects/${id}/research-exports`,{base_revision:revision});
      if(project.project_id!==id||generation!==g||project.revision!==revision)throw new Error("工程已变化，请重新导出。");
      const response=await fetch(`/api/projects/${id}/research-exports/${receipt.export_id}`,{headers:{Authorization:"Bearer "+token}});
      if(!response.ok){const error=await response.json();throw new Error(error.detail||"研究包读取失败");}
      const data=await response.arrayBuffer(),hash=Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256",data)),v=>v.toString(16).padStart(2,"0")).join("");
      if(hash!==receipt.sha256||response.headers.get("X-Mapforge-Purpose")!=="RESEARCH_ONLY"||data.byteLength!==receipt.size_bytes)throw new Error("下载字节校验失败，未保存研究包。");
      if(project.project_id!==id||generation!==g||project.revision!==revision)throw new Error("下载期间工程已变化，请重新导出。");
      const url=URL.createObjectURL(new Blob([data],{type:"application/zip"})),link=make("a");link.href=url;link.download=receipt.file_name;document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),60000);
      $("export-status").textContent=`研究包已生成并发起下载 · ${(data.byteLength/1024).toFixed(1)} KB · 裁决 ${receipt.decision}。完整检查和阻断项均随包保留。`;
    }catch(e){if(project?.project_id===id&&generation===g)$("export-status").textContent="未导出："+e.message;throw e;}
  });
  return {buttons,changed,selectionChanged:clear,escape,point,draw:overlay,pick};
})();
