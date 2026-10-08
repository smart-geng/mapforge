// Actual app and surface pane with delayed HTTP responses. These contracts
// prove frontend state ownership, not geometry or independent user acceptance.
import test from "node:test";
import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {createHash,webcrypto} from "node:crypto";
import vm from "node:vm";

const app=readFileSync(new URL("../mapforge/workbench/static/app.js",import.meta.url),"utf8");
const script=readFileSync(new URL("../mapforge/workbench/static/surface-editing.js",import.meta.url),"utf8");
const TYPE="source_supported_surface_tracks_v2",tick=()=>new Promise(resolve=>setImmediate(resolve));
const template=()=>({type:TYPE,source_ref:"source-junction",scope:{capability_id:"0621-source-supported-surface-v1",feature_ids:["source-junction","source-lane"]},parameters:{source_content_hash:"s".repeat(64),operation_hash:"o".repeat(64)}});
const descriptor=(enabled=true)=>({supported:true,enabled,reason:"来源范围已核验",command_template:template(),capability:{actions:["撤回无来源支撑扫掠","依原始内边界补全分隔带","按来源面重新表达辅助铺面"]}});
const checks=()=>[{gate:"whole-map-validation",status:"COMPLETED",tiers:{T1:{status:"FAIL"},T2:{status:"FAIL"}}},{gate:"production-delivery",status:"BLOCKED"}];
const candidate=()=>({candidate_id:"fresh-candidate",checks:checks(),target_content_hash:"saved-content",context:{compiler_hash:"c"}});
function project(id="p1"){return {project_id:id,name:id,revision:1,draft_epoch:1,content_hash:"base-content",context:{compiler_hash:"c"},source_snapshot:{objects:[],frame:{},issues:[]},intents:[],candidate:null,validation:null,status:{read_only:false,candidate_stale:false,validation_stale:false,source_issues:[],can_undo:false,can_redo:false}};}
const runningJob=(id="compile-1",operation="compile_surface",base=1,content="base-content")=>({job_id:id,operation,state:"running",base_revision:base,content_hash:content,historical:false,archive:{state:"persisted",recorded_state:"running"}});
const completedJob=()=>({...runningJob(),state:"succeeded",archive:{state:"persisted",recorded_state:"succeeded"},result:{status:"COMPILED",candidate:candidate(),evidence:{candidate_sha256:"a".repeat(64)}}});
const polygon=()=>({type:"Polygon",coordinates:[[[0,0],[20,0],[20,20],[0,20],[0,0]],[[5,5],[10,5],[10,10],[5,10],[5,5]]]});
function geometry(accepted=false,id="p1"){return {available:true,candidate_id:"fresh-candidate",candidate_stale:false,candidate_accepted:accepted,preview:!accepted,formal_export_available:false,report:{layers:[{id:"source",title:"来源面",geometry:polygon()},{id:"candidate",title:"实际辅助面",geometry:polygon()}],issues:[{id:"void",title:"来源空区",detail:"保留，不能直接填面",geometry:polygon()}],context:{project_id:id,candidate_id:"fresh-candidate",candidate_sha256:"a".repeat(64),source_snapshot_id:"snapshot",source_content_hash:"s".repeat(64),candidate_accepted:accepted,formal_release_verified:false,frame:{kind:"local-eqc",unit:"m",origin:[106.2,29.6],absolute_crs_status:"unverified"}}}};}
function fidelity(){
  const lane=(id,p95,missing=false)=>({id:"fidelity:"+id,source_lane_id:id,policy_class:"shp.field-approach",target:{road_id:"10",side:"right",lane_id:-Number(id),section_first:0,section_last:1},source_to_target:{p95_m:p95,max_m:p95+.1,median_m:p95/2,count:12},target_to_source:{p95_m:p95/2,max_m:p95,median_m:p95/3,count:12},stop_line:{availability:missing?"unavailable":"not-applicable",reason:missing?"source-stopline-not-linked":null},source_geometry:{type:"LineString",coordinates:[[Number(id)*100,0],[Number(id)*100+20,0]]},target_geometry:{type:"LineString",coordinates:[[Number(id)*100,1],[Number(id)*100+20,1]]},issues:missing?[{code:"stopline-unmeasurable",reason:"source-stopline-not-linked",message:"来源停止线不可测；不能按邻近位置补绑。"}]:[]});
  const lanes=[lane("1",.1),lane("2",.3,true),lane("3",.2)];
  for(const item of lanes){item.primary_source_ref={id:"raw:"+item.source_lane_id,role:"lane",source_ref:{snapshot_id:"snapshot",layer:"IBD_LANE_LINK",record_index:Number(item.source_lane_id),part_index:0}};item.source_refs=[item.primary_source_ref,{id:"merge:"+item.source_lane_id,role:"lane_merge",source_ref:{snapshot_id:"snapshot",layer:"IBD_LANE_LINK_MERGE",record_index:Number(item.source_lane_id)+20,part_index:null}}];}
  return {schema:"mapforge/workbench-fidelity-inspection/v1",status:"AVAILABLE",gate_status:"UNAVAILABLE",scope:{matched_source_lanes:3,eligible_source_lanes:3,target_components:3},summary:{lane_count:3,stopline_unavailable_count:1},lanes};
}
function geometryWithFidelity(value=fidelity()){const result=geometry();result.report.fidelity=value;return result;}

async function environment(){
  const elements=new Map(),requests=[],listeners=new Map(),timers=[],calls=[],downloads=[];
  class Element{
    constructor(tag="div"){Object.assign(this,{tag,children:[],textContent:"",value:"",disabled:false,hidden:true,open:false,attributes:{},style:{},dataset:{},clientWidth:900,clientHeight:500,classList:{toggle(){}}});}
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}get firstChild(){return this.children[0];}
    setAttribute(name,value){this.attributes[name]=value;}addEventListener(name,handler){this["on"+name]=handler;}
    showModal(){this.open=true;}close(){this.open=false;}setPointerCapture(){}remove(){}
    click(){if(this.disabled)return;if(this.tag==="a")downloads.push({href:this.href,name:this.download});return this.onclick?.();}
    getContext(){const id=this.id;return Object.fromEntries(["setTransform","clearRect","beginPath","moveTo","lineTo","closePath","fill","stroke","fillText","arc"].map(name=>[name,(...args)=>calls.push({id,name,args})]));}
  }
  const element=id=>{if(!elements.has(id)){const item=new Element();item.id=id;elements.set(id,item);}return elements.get(id);};
  const response=body=>({ok:true,json:async()=>body});
  const sandbox={console,URLSearchParams,crypto:webcrypto,Blob,devicePixelRatio:1,location:{hash:"",pathname:"/"},history:{replaceState(){}},
    sessionStorage:{getItem:()=>"test-token",setItem(){}},document:{getElementById:element,createElement:tag=>new Element(tag),createTextNode:text=>({textContent:text}),body:new Element("body"),addEventListener:(name,handler)=>listeners.set(name,handler)},window:{addEventListener(){}},
    ResizeObserver:class{observe(){}},confirm:()=>true,URL:{createObjectURL:()=>"blob:research",revokeObjectURL(){}},
    setTimeout:(handler,delay)=>{timers.push({handler,delay});return timers.length;},
    fetch(url,options={}){if(url==="/api/catalog")return Promise.resolve(response({junctions:[],notice:"test"}));if(url==="/api/projects"||url.endsWith("/jobs"))return Promise.resolve(response([]));
      return new Promise((resolve,reject)=>{const request={url,options,settled:false,respond(body){this.settled=true;resolve(response(body));},raw(value){this.settled=true;resolve(value);},reject(error){this.settled=true;reject(error);}};requests.push(request);});}};
  vm.createContext(sandbox);const run=code=>vm.runInContext(code,sandbox);run(app);run(script);await tick();
  const e={element,requests,sandbox,run,listeners,timers,calls,downloads};
  e.load=(value=project())=>run(`generation++;applyProject(${JSON.stringify(value)},true);`);
  e.pending=fragment=>{const found=requests.findLast(item=>!item.settled&&item.url.includes(fragment));assert.ok(found,"missing request "+fragment);return found;};
  e.open=()=>element("open-surface-editing").click();
  e.ready=async(enabled=true,value=project())=>{e.load(value);const task=e.open();e.pending("/surface-rebuild").respond(descriptor(enabled));await task;};
  e.startPreview=async()=>{const task=element("surface-preview").click();const request=e.pending("/surface-rebuild/preview"),body=JSON.parse(request.options.body);e.command=body.command;e.ticket={command:body.command,base_revision:1,target_revision:2,target_draft_epoch:2,target_content_hash:"saved-content"};request.respond({job:runningJob(),preview:e.ticket});await task;};
  e.finishPreview=async(job=completedJob(),value=geometry())=>{e.pending("/jobs/compile-1").respond(job);await tick();if(job.result?.status==="COMPILED"&&!job.historical&&job.archive.state==="persisted"){e.pending("/surface-rebuild/geometry").respond(value);await tick();}};
  e.savedProject=()=>({...project(),revision:2,draft_epoch:2,content_hash:"saved-content",intents:[e.command],status:{...project().status,can_undo:true}});
  e.save=async()=>{const task=element("surface-save").click();e.pending("/commands").respond(e.savedProject());await tick();const pending=requests.findLast(item=>!item.settled&&item.url.includes("/geometry"));if(pending)pending.respond(geometry());await task;};
  e.accept=async()=>{const task=element("surface-accept").click();e.acceptedProject={...e.savedProject(),revision:3,candidate:{...candidate(),accepted_epoch:2,accepted_revision:3}};e.pending("/surface-rebuild/accept").respond(e.acceptedProject);await tick();e.pending("/geometry").respond(geometry(true));await task;};
  e.confirmed=async()=>{await e.ready();await e.startPreview();await e.finishPreview();await e.save();await e.accept();};
  e.check=async()=>{const task=element("surface-check").click();e.pending("/checking/start").respond({job:runningJob("check-1","validate",3,"saved-content")});await task;
    e.validation={bundle_id:"validation-1",candidate_id:"fresh-candidate",decision:"BLOCKED",checks:checks()};e.pending("/jobs/check-1").respond({...runningJob("check-1","validate",3,"saved-content"),state:"succeeded",archive:{state:"persisted",recorded_state:"succeeded"},result:{status:"VALIDATED",validation:e.validation}});await tick();};
  e.attach=async()=>{const task=element("surface-attach").click();e.checkedProject={...e.acceptedProject,revision:4,validation:e.validation};e.pending("/checking/attach").respond(e.checkedProject);await tick();e.pending("/geometry").respond(geometry(true));await task;};
  return e;
}

test("unsupported source is explicit and offers no geometry mutations",async()=>{
  const e=await environment();assert.equal(e.element("open-surface-editing").disabled,true);e.load();const task=e.open();e.pending("/surface-rebuild").respond({supported:false,enabled:false,reason:"此源尚无登记能力"});await task;
  assert.equal(e.element("surface-reason").textContent,"此源尚无登记能力");for(const id of ["surface-preview","surface-save","surface-compile","surface-accept","surface-check","surface-export"])assert.equal(e.element(id).disabled,true);
  assert.equal(e.requests.length,1);
});

test("switching to an unsupported source ends loading and clears previous checks and actions",async()=>{
  const e=await environment();await e.confirmed();assert.ok(e.element("surface-checks").children.length>0);assert.equal(e.element("surface-actions").children.length,3);
  e.element("surface-close").click();e.load(project("p2"));const task=e.open();e.pending("/surface-rebuild").respond({supported:false,enabled:false,reason:"当前仅登记 0621 来源重建"});await task;
  assert.equal(e.element("surface-status").textContent,"来源重建不可用：当前仅登记 0621 来源重建");
  assert.doesNotMatch(e.element("surface-status").textContent,/正在读取/);assert.equal(e.element("surface-checks").children.length,0);assert.equal(e.element("surface-actions").children.length,0);assert.equal(e.element("surface-binding").textContent,"");assert.equal(e.element("surface-preview").disabled,true);
});

test("enabling records capability only, refreshes the descriptor, and exposes three fixed actions",async()=>{
  const e=await environment();await e.ready(false);assert.equal(e.element("surface-enable").hidden,false);
  const task=e.element("surface-enable").click();const request=e.pending("/enable");assert.deepEqual(Object.keys(JSON.parse(request.options.body)).sort(),["base_revision","command_id"]);
  request.respond({project:project(),capability:descriptor().capability});await tick();e.pending("/surface-rebuild").respond(descriptor());await task;
  assert.equal(e.element("surface-actions").children.length,3);assert.equal(e.run("project.intents.length"),0);assert.equal(e.element("surface-enable").hidden,true);
});

test("preview renders only the fresh job; confirmation uses its exact command once; accepted means research",async()=>{
  const e=await environment();await e.ready();const before=e.run("JSON.stringify(project)");await e.startPreview();assert.equal(e.run("JSON.stringify(project)"),before);
  assert.equal(e.element("surface-save").disabled,false);assert.equal(e.element("surface-accept").disabled,true);await e.finishPreview();
  assert.match(e.element("surface-geometry-state").textContent,/从原件生成.*未接受/);assert.ok(e.requests.some(item=>item.url.endsWith("/geometry?job_id=compile-1")));
  assert.equal(e.calls.filter(item=>item.id==="surface-map"&&item.name==="fill").every(item=>item.args[0]==="evenodd"),true);
  const saveTask=e.element("surface-save").click();const savedRequest=e.pending("/commands");assert.deepEqual(JSON.parse(savedRequest.options.body).command,e.command);savedRequest.respond(e.savedProject());await tick();e.pending("/geometry").respond(geometry());await saveTask;
  assert.equal(e.element("surface-preview").disabled,true);assert.equal(e.element("surface-save").disabled,true);assert.equal(e.element("surface-accept").disabled,false);
  await e.accept();assert.equal(e.run("project.candidate.candidate_id"),"fresh-candidate");assert.match(e.element("surface-geometry-state").textContent,/已接受.*阻断/);assert.equal(e.element("surface-accept").disabled,true);assert.equal(e.element("surface-check").disabled,false);
  assert.ok(e.element("surface-checks").children.some(item=>item.textContent.includes("T1 FAIL")&&item.textContent.includes("T2 FAIL")));
});

test("rejected preview may save the intended draft but never enables acceptance",async()=>{
  const e=await environment();await e.ready();await e.startPreview();await e.finishPreview({...completedJob(),result:{status:"REJECTED",error:{message:"真实来源守卫拒绝"}}});
  assert.match(e.element("surface-status").textContent,/真实来源守卫拒绝/);assert.equal(e.element("surface-accept").disabled,true);await e.save();assert.equal(e.run("project.intents.length"),1);assert.equal(e.element("surface-compile").disabled,false);assert.equal(e.run("project.candidate"),null);
});

test("an existing unique saved intent compiles without appending a second command",async()=>{
  const e=await environment(),intent={...template(),command_id:"saved-operation"};
  await e.ready(true,{...project(),revision:7,draft_epoch:4,content_hash:"saved-content",intents:[intent]});
  assert.equal(e.element("surface-preview").disabled,true);assert.equal(e.element("surface-save").disabled,true);assert.equal(e.element("surface-compile").disabled,false);
  const task=e.element("surface-compile").click(),request=e.pending("/surface-rebuild/compile");assert.equal(JSON.parse(request.options.body).base_revision,7);
  request.respond({job:runningJob("compile-1","compile_surface",7,"saved-content")});await task;await e.finishPreview();
  assert.equal(e.element("surface-accept").disabled,false);assert.equal(e.run("project.intents.length"),1);assert.equal(e.requests.filter(item=>item.url.endsWith("/commands")).length,0);
});

test("a lost preview response retries the same request and exact command identity",async()=>{
  const e=await environment();await e.ready();let task=e.element("surface-preview").click();const first=e.pending("/preview"),body=first.options.body;first.reject(new Error("connection lost after start"));await task;
  assert.equal(e.requests.filter(item=>item.url.includes("/jobs/")).length,0);task=e.element("surface-preview").click();const retry=e.pending("/preview");assert.equal(retry.options.body,body);
  const command=JSON.parse(body).command;retry.respond({job:runningJob(),preview:{command,base_revision:1,target_revision:2,target_draft_epoch:2,target_content_hash:"saved-content"}});await task;
  assert.ok(e.pending("/jobs/compile-1"));
});

test("a lost acceptance response retries its same transaction identity",async()=>{
  const e=await environment();await e.ready();await e.startPreview();await e.finishPreview();await e.save();let task=e.element("surface-accept").click();const first=e.pending("/accept"),body=first.options.body;first.reject(new Error("receipt lost"));await task;
  task=e.element("surface-accept").click();const retry=e.pending("/accept");assert.equal(retry.options.body,body);retry.respond({...e.savedProject(),revision:3,candidate:{...candidate(),accepted_epoch:2,accepted_revision:3}});await tick();e.pending("/geometry").respond(geometry(true));await task;assert.equal(e.element("surface-accept").disabled,true);
});

test("a late mutation result after a project switch cannot replace the new project",async()=>{
  const e=await environment();await e.ready();const task=e.element("surface-preview").click(),pending=e.pending("/preview");e.load(project("p2"));pending.respond({job:runningJob(),preview:{command:{...template(),command_id:"old"},base_revision:1,target_revision:2,target_draft_epoch:2,target_content_hash:"saved-content"}});await task;
  assert.equal(e.run("project.project_id"),"p2");assert.equal(e.requests.filter(item=>item.url.includes("/jobs/")).length,0);assert.equal(e.element("surface-fit").disabled,true);
});

test("switching away after completed geometry removes its binding and canvas controls",async()=>{
  const e=await environment();await e.ready();await e.startPreview();await e.finishPreview();assert.notEqual(e.element("surface-binding").textContent,"");e.load(project("p2"));
  assert.equal(e.element("surface-binding").textContent,"");assert.equal(e.element("surface-fit").disabled,true);assert.equal(e.element("surface-preview").disabled,true);
});

test("close and reopen resume the same running job without a new compilation request",async()=>{
  const e=await environment();await e.ready();await e.startPreview();const oldPoll=e.pending("/jobs/compile-1");e.element("surface-close").click();oldPoll.respond(completedJob());await tick();
  assert.equal(e.element("surface-editing-dialog").open,false);const task=e.open();e.pending("/surface-rebuild").respond(descriptor());await task;
  assert.ok(e.pending("/jobs/compile-1"));assert.equal(e.requests.filter(item=>item.url.endsWith("/preview")).length,1);assert.equal(e.requests.filter(item=>item.url.endsWith("/cancel")).length,0);
});

for(const action of ["project","revision","close"]){
  for(const outcome of ["success","error"]){
    test(`late descriptor ${outcome} after ${action} never enables old controls`,async()=>{
      const e=await environment();e.load();const task=e.open(),pending=e.pending("/surface-rebuild");
      if(action==="project")e.load(project("p2"));if(action==="revision")e.run("project={...project,revision:2};applyProject(project);");if(action==="close")e.element("surface-close").click();
      if(outcome==="success")pending.respond(descriptor());else pending.reject(new Error("OLD response"));await task;
      assert.equal(e.element("surface-preview").disabled,true);assert.equal(e.requests.length,1);
    });
  }
}

test("undo and redo invalidate the old preview even when draft contents return",async()=>{
  const e=await environment();await e.ready();await e.startPreview();await e.finishPreview();await e.save();
  e.run(`applyProject(${JSON.stringify({...project(),revision:3,draft_epoch:3})});`);
  e.run(`applyProject(${JSON.stringify({...e.savedProject(),revision:4,draft_epoch:4})});`);
  assert.equal(e.element("surface-accept").disabled,true);assert.equal(e.element("surface-fit").disabled,true);assert.equal(e.run("project.intents.length"),1);
});

test("old geometry after a project switch cannot enter the current pane",async()=>{
  const e=await environment();await e.ready();await e.startPreview();e.pending("/jobs/compile-1").respond(completedJob());await tick();const old=e.pending("/geometry");e.load(project("p2"));old.respond(geometry());await tick();
  assert.equal(e.element("surface-fit").disabled,true);assert.equal(e.element("surface-binding").textContent,"");
});

test("cancellation defeats a previously buffered successful poll",async()=>{
  const e=await environment();await e.ready();await e.startPreview();const poll=e.pending("/jobs/compile-1");
  const task=e.element("surface-cancel").click();e.pending("/cancel").respond({...runningJob(),state:"cancelled"});await task;poll.respond(completedJob());await tick();
  assert.equal(e.element("surface-accept").disabled,true);assert.match(e.element("surface-status").textContent,/取消/);assert.equal(e.requests.filter(item=>item.url.includes("/geometry")).length,0);
});

for(const change of [job=>job.historical=true,job=>job.archive.state="failed",job=>job.archive.recorded_state="running",job=>job.state="timed_out"]){
  test(`nonacceptable terminal job does not enable acceptance: ${change}`,async()=>{
    const e=await environment();await e.ready();await e.startPreview();const job=completedJob();change(job);e.pending("/jobs/compile-1").respond(job);await tick();assert.equal(e.element("surface-accept").disabled,true);assert.equal(e.requests.filter(item=>item.url.includes("/geometry")).length,0);
  });
}

for(const mutate of [value=>value.candidate_id="old-candidate",value=>value.report.context.project_id="p2",value=>value.report.context.frame.unit="degrees",value=>value.report.context.formal_release_verified=true,value=>value.candidate_stale=true]){
  test(`geometry ownership and frame guard rejects ${mutate}`,async()=>{
    const e=await environment();await e.ready();await e.startPreview();const value=geometry();mutate(value);await e.finishPreview(completedJob(),value);assert.equal(e.element("surface-fit").disabled,true);assert.match(e.element("surface-geometry-state").textContent,/不匹配/);
  });
}

test("whole-map checks must be attached separately and retain FAIL/BLOCKED",async()=>{
  const e=await environment();await e.confirmed();await e.check();assert.equal(e.run("project.validation"),null);assert.equal(e.element("surface-attach").disabled,false);assert.equal(e.element("surface-export").disabled,true);
  await e.attach();assert.equal(e.run("project.validation.decision"),"BLOCKED");assert.equal(e.element("surface-export").disabled,false);assert.equal(e.element("surface-attach").disabled,true);
  assert.ok(e.element("surface-checks").children.some(item=>item.textContent.includes("T2 FAIL")));
});

test("engineers can distinguish blocked gates and unavailable baseline without losing original statuses",async()=>{
  const e=await environment();await e.confirmed();const task=e.element("surface-check").click();e.pending("/checking/start").respond({job:runningJob("check-1","validate",3,"saved-content")});await task;
  const validation={decision:"BLOCKED",checks:[
    {gate:"G8",status:"UNAVAILABLE"},{gate:"G11",status:"FAIL"},{gate:"G11-edge-contacts",status:"FAIL"},
    {gate:"baseline-comparison",status:"UNAVAILABLE",reason:"unavailable-original-generation-failed"},
    {gate:"delivery-decision",status:"BLOCKED",blocked_reasons:[
      {code:"crs_not_absolutely_verified",integrity:"internally-consistent"},
      {code:"required_gate_not_pass",gate_id:"G11",gate_status:"FAIL"},
      {code:"required_gate_not_pass",gate_id:"G11-edge-contacts",gate_status:"FAIL"},
      {code:"required_gate_not_pass",gate_id:"G8",gate_status:"UNAVAILABLE"}]}]};
  const before=JSON.stringify(validation);
  e.pending("/jobs/check-1").respond({...runningJob("check-1","validate",3,"saved-content"),state:"succeeded",archive:{state:"persisted",recorded_state:"succeeded"},result:{status:"VALIDATED",validation}});await tick();
  const rendered=e.element("surface-checks").children.map(item=>item.textContent).join("\n");
  assert.match(rendered,/绝对坐标尚未经过独立核验/);
  assert.match(rendered,/结构与几何（G11）未满足交付要求：FAIL（未通过）/);
  assert.match(rendered,/车道边缘接口（G11-edge-contacts）未满足交付要求：FAIL（未通过）/);
  assert.match(rendered,/来源保真（G8）未满足交付要求：UNAVAILABLE（不可评估）/);
  assert.match(rendered,/原流程完整生成失败，不能进行完整基线比较/);
  assert.match(rendered,/完整基线比较（baseline-comparison）：UNAVAILABLE/);
  assert.match(rendered,/交付裁决 BLOCKED（阻断）/);assert.equal(JSON.stringify(validation),before);
});

test("check result from an older project revision cannot be saved",async()=>{
  const e=await environment();await e.confirmed();const task=e.element("surface-check").click();e.pending("/checking/start").respond({job:runningJob("check-1","validate",3,"saved-content")});await task;const pending=e.pending("/jobs/check-1");
  e.run(`applyProject(${JSON.stringify({...e.acceptedProject,revision:4})});`);pending.respond({...runningJob("check-1","validate",3,"saved-content"),state:"succeeded",archive:{state:"persisted",recorded_state:"succeeded"},result:{status:"VALIDATED",validation:{decision:"BLOCKED",checks:checks()}}});await tick();assert.equal(e.element("surface-attach").disabled,true);
});

const zip=new TextEncoder().encode("actual research package contract bytes");
function receipt(){return {export_id:"research-1",file_name:"RESEARCH_ONLY.zip",size_bytes:zip.length,sha256:createHash("sha256").update(zip).digest("hex")};}
function download(purpose="RESEARCH_ONLY"){return {ok:true,headers:{get:()=>purpose},arrayBuffer:async()=>zip.buffer.slice(zip.byteOffset,zip.byteOffset+zip.byteLength)};}

test("verified research download preserves current project and explicitly retains BLOCKED",async()=>{
  const e=await environment();await e.confirmed();await e.check();await e.attach();const before=e.run("JSON.stringify(project)");const task=e.element("surface-export").click();e.pending("/research-exports").respond(receipt());await tick();e.pending("/research-exports/research-1").raw(download());await task;
  assert.deepEqual(e.downloads,[{href:"blob:research",name:"RESEARCH_ONLY.zip"}]);assert.equal(e.run("JSON.stringify(project)"),before);assert.match(e.element("surface-status").textContent,/FAIL\/BLOCKED/);
});

for(const failure of ["hash","purpose","version"]){
  test(`research download refuses ${failure} mismatch`,async()=>{
    const e=await environment();await e.confirmed();await e.check();await e.attach();const task=e.element("surface-export").click(),value=receipt();if(failure==="hash")value.sha256="0".repeat(64);e.pending("/research-exports").respond(value);await tick();if(failure==="version")e.run("project.revision++;");e.pending("/research-exports/research-1").raw(download(failure==="purpose"?"OTHER":"RESEARCH_ONLY"));await task;assert.equal(e.downloads.length,0);
  });
}

test("modal Escape never clears an unconfirmed note in the source workspace",async()=>{
  const e=await environment();await e.ready();e.element("note").value="保留未确认待办";e.element("note").oninput();e.listeners.get("keydown")({key:"Escape",preventDefault(){}});assert.equal(e.run("pendingNote"),true);assert.equal(e.element("note").value,"保留未确认待办");
});

test("fidelity keeps original G8 unavailable while sorting all lane observations and separating stopline gaps",async()=>{
  const e=await environment();await e.ready();await e.startPreview();const value=geometryWithFidelity(),before=JSON.stringify(value);await e.finishPreview(completedJob(),value);
  const lanes=e.element("surface-fidelity-lanes").children,stops=e.element("surface-fidelity-stops").children;
  assert.deepEqual(lanes.map(item=>item.dataset.laneId),["fidelity:2","fidelity:3","fidelity:1"]);assert.deepEqual(stops.map(item=>item.dataset.laneId),["fidelity:2"]);
  assert.match(e.element("surface-fidelity-status").textContent,/原 G8：UNAVAILABLE/);assert.match(e.element("surface-fidelity-status").textContent,/不判单车道通过或失败/);assert.equal(JSON.stringify(value),before);assert.equal(e.element("surface-fit").disabled,false);
});

test("fidelity selection shows only one paired lane and zooms without modifying project or invoking writes",async()=>{
  const e=await environment();await e.ready();await e.startPreview();await e.finishPreview(completedJob(),geometryWithFidelity());const before=e.run("JSON.stringify(project)"),requests=e.requests.length;
  const lanes=e.element("surface-fidelity-lanes").children;lanes[0].click();assert.match(e.element("surface-fidelity-selection").textContent,/来源车道 2/);assert.equal(lanes[0].attributes["aria-pressed"],"true");assert.equal(e.element("surface-fidelity-stops").children[0].attributes["aria-pressed"],"true");
  const begin=e.calls.length;lanes[1].click();const drawn=e.calls.slice(begin).filter(item=>item.id==="surface-map");
  // Two polygon layers, one polygon issue, exactly two selected lines, and scale bar.
  assert.equal(drawn.filter(item=>item.name==="stroke").length,6);assert.equal(lanes[0].attributes["aria-pressed"],"false");assert.equal(lanes[1].attributes["aria-pressed"],"true");
  assert.match(e.element("surface-fidelity-selection").textContent,/来源车道 3/);assert.match(e.element("surface-fidelity-primary").textContent,/IBD_LANE_LINK · 记录 3 · part 0（索引从 0 开始）/);assert.match(e.element("surface-fidelity-refs").children[0].textContent,/IBD_LANE_LINK_MERGE · 记录 23 · part 无几何/);assert.equal(e.run("JSON.stringify(project)"),before);assert.equal(e.requests.length,requests);
  e.element("surface-fidelity-clear").click();assert.equal(e.element("surface-fidelity-clear").disabled,true);assert.equal(lanes[1].attributes["aria-pressed"],"false");assert.equal(e.element("surface-fidelity-lanes").children.length,3);assert.equal(e.element("surface-issues").children.length,1);
});

test("fidelity preserves small positive distances instead of displaying false zero",async()=>{
  const e=await environment();await e.ready();await e.startPreview();const value=fidelity();
  value.lanes[0].source_to_target.p95_m=.0003098236;value.lanes[0].target_to_source.p95_m=1e-9;
  value.lanes[1].source_to_target.p95_m=0;value.lanes[1].target_to_source.p95_m=.001234;
  await e.finishPreview(completedJob(),geometryWithFidelity(value));
  const lanes=e.element("surface-fidelity-lanes").children,small=lanes.find(item=>item.dataset.laneId==="fidelity:1").children[0].textContent,zero=lanes.find(item=>item.dataset.laneId==="fidelity:2").children[0].textContent;
  assert.equal(small,"来源→候选 P95 0.000310 m · 候选→来源 P95 1.00e-9 m");assert.doesNotMatch(small,/\b0\.000 m/);
  assert.equal(zero,"来源→候选 P95 0.000 m · 候选→来源 P95 0.001 m");
});

for(const mutate of [value=>delete value.lanes[0].target_geometry,value=>value.lanes[1].source_geometry.coordinates[0][0]=NaN,value=>value.lanes[0].source_to_target.p95_m=".1",value=>value.summary.lane_count=2,value=>value.lanes[1].id=value.lanes[0].id,value=>value.lanes[0].target.section_last=-1,value=>delete value.lanes[0].primary_source_ref,value=>value.lanes[1].source_refs[0].source_ref.snapshot_id="other-source"]){
  test(`malformed fidelity clears its pane and leaves paving available: ${mutate}`,async()=>{
    const e=await environment();await e.ready();await e.startPreview();const value=fidelity();mutate(value);await e.finishPreview(completedJob(),geometryWithFidelity(value));
    assert.match(e.element("surface-fidelity-status").textContent,/不可用/);assert.equal(e.element("surface-fidelity-lanes").children.length,0);assert.equal(e.element("surface-fidelity-clear").disabled,true);assert.equal(e.element("surface-fit").disabled,false);assert.equal(e.element("surface-issues").children.length,1);
  });
}

for(const mode of ["close","project","revision","refresh"]){
  test(`fidelity selection cannot survive ${mode}`,async()=>{
    const e=await environment();await e.ready();await e.startPreview();await e.finishPreview(completedJob(),geometryWithFidelity());e.element("surface-fidelity-lanes").children[0].click();assert.equal(e.element("surface-fidelity-clear").disabled,false);
    if(mode==="close")e.element("surface-close").click();
    if(mode==="project")e.load(project("p2"));
    if(mode==="revision")e.run("applyProject({...project,revision:project.revision+1});");
    if(mode==="refresh"){const task=e.element("surface-save").click();e.pending("/commands").respond(e.savedProject());await tick();assert.equal(e.element("surface-fidelity-clear").disabled,true);e.pending("/geometry").respond(geometryWithFidelity({status:"UNAVAILABLE",reason:"新的比较范围不可用"}));await task;assert.match(e.element("surface-fidelity-status").textContent,/新的比较范围不可用/);}
    assert.equal(e.element("surface-fidelity-clear").disabled,true);assert.equal(e.element("surface-fidelity-lanes").children.length,0);assert.equal(e.element("surface-fidelity-stops").children.length,0);assert.equal(e.element("surface-fidelity-primary").textContent,"");assert.equal(e.element("surface-fidelity-refs").children.length,0);assert.equal(e.element("surface-fidelity-selection").textContent,"未选中比较车道");
  });
}

test("late geometry cannot repopulate fidelity after close",async()=>{
  const e=await environment();await e.ready();await e.startPreview();e.pending("/jobs/compile-1").respond(completedJob());await tick();const pending=e.pending("/geometry");e.element("surface-close").click();pending.respond(geometryWithFidelity());await tick();assert.equal(e.element("surface-fidelity-lanes").children.length,0);assert.equal(e.element("surface-fidelity-clear").disabled,true);
});
