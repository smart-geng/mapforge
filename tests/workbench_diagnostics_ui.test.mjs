// Real app and diagnostics scripts with delayed HTTP and an observable canvas.
// These checks verify UI ownership and read-only behavior, not browser layout.
import test from "node:test";
import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {webcrypto} from "node:crypto";
import vm from "node:vm";

const app=readFileSync(new URL("../mapforge/workbench/static/app.js",import.meta.url),"utf8");
const diagnostics=readFileSync(new URL("../mapforge/workbench/static/diagnostics.js",import.meta.url),"utf8");
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const polygon=(x=0)=>({type:"Polygon",coordinates:[[[x,0],[x+20,0],[x+20,20],[x,20],[x,0]],[[x+5,5],[x+10,5],[x+10,10],[x+5,10],[x+5,5]]]});
const report=(title="来源空区")=>({schema:"mapforge/surface-diagnostics/v1",status:"RESEARCH_ONLY",
  context:{candidate_sha256:"a".repeat(64),source_snapshot_id:"snapshot",source_content_hash:"b".repeat(64),
    candidate_accepted:false,formal_release_verified:false,
    frame:{kind:"local-eqc",unit:"m",origin:[106.2,29.6],absolute_crs_status:"unverified"}},
  layers:[{id:"source",title:"来源面",geometry:polygon()},{id:"auxiliary",title:"辅助面",geometry:{type:"MultiPolygon",coordinates:[polygon(30).coordinates,polygon(60).coordinates]}}],
  issues:[{id:"source-void",kind:"source_void",title,detail:"保留原来源空区；不能自动补面。",area_m2:25,road_ids:["12"],source_lane_ids:["original-lane"],geometry:polygon(100)}]});
function project(id="p1"){return {project_id:id,name:id,revision:7,content_hash:"draft",draft_epoch:1,
  source_snapshot:{objects:[],frame:{},issues:[]},intents:[],candidate:{candidate_id:"accepted-other-candidate"},
  validation:{decision:"BLOCKED"},status:{read_only:false,candidate_stale:false,validation_stale:false,source_issues:[]}};}

async function environment(){
  const elements=new Map(),requests=[],listeners=new Map(),calls=[];
  class Element{
    constructor(tag="div"){Object.assign(this,{tag,children:[],textContent:"",value:"",disabled:false,hidden:true,open:false,
      attributes:{},dataset:{},style:{},clientWidth:900,clientHeight:500,classList:{toggle(){}}});}
    setAttribute(key,value){this.attributes[key]=value;}
    replaceChildren(...items){this.children=items;}
    append(...items){this.children.push(...items);}
    get firstChild(){return this.children[0];}
    addEventListener(type,fn){this["on"+type]=fn;}
    showModal(){this.open=true;}
    close(){this.open=false;}
    setPointerCapture(){}
    getContext(){const id=this.id;return Object.fromEntries(["setTransform","clearRect","beginPath","moveTo","lineTo","closePath","fill","stroke","fillText","arc"].map(name=>[name,(...args)=>calls.push({id,name,args})]));}
    click(){if(!this.disabled)return this.onclick?.();}
  }
  const element=id=>{if(!elements.has(id)){const value=new Element();value.id=id;elements.set(id,value);}return elements.get(id);};
  const sandbox={console,URLSearchParams,crypto:webcrypto,devicePixelRatio:1,location:{hash:"",pathname:"/"},history:{replaceState(){}},
    sessionStorage:{getItem:()=>"test-token",setItem(){}},
    document:{getElementById:element,createElement:tag=>new Element(tag),createTextNode:value=>({textContent:value}),addEventListener:(name,fn)=>listeners.set(name,fn)},
    window:{addEventListener(){}},ResizeObserver:class{observe(){}},confirm:()=>true,setTimeout(){},
    fetch(url,options={}){const response=body=>({ok:true,json:async()=>body});
      if(url==="/api/catalog")return Promise.resolve(response({junctions:[],notice:"catalog"}));
      if(url==="/api/projects")return Promise.resolve(response([]));
      return new Promise((resolve,reject)=>requests.push({url,options,reject,respond:body=>resolve(response(body))}));}};
  vm.createContext(sandbox);const run=code=>vm.runInContext(code,sandbox);run(app);run(diagnostics);await tick();
  const e={element,requests,calls,sandbox,run,listeners};
  e.load=(id="p1")=>run(`generation++;applyProject(${JSON.stringify(project(id))},true);`);
  e.open=()=>{const task=element("open-diagnostics").click();return {task,request:requests.at(-1)};};
  e.show=async(value=report())=>{const pending=e.open();pending.request.respond({available:true,report:value});await pending.task;};
  e.assertEmpty=()=>{assert.equal(element("diagnostics-issues").children.length,0);assert.equal(element("diagnostics-binding").textContent,"");assert.equal(element("diagnostics-fit").disabled,true);};
  return e;
}

test("only an open project enables diagnostics, and existing busy locks remain authoritative",async()=>{
  const e=await environment();assert.equal(e.element("open-diagnostics").disabled,true);e.load();assert.equal(e.element("open-diagnostics").disabled,false);
  e.run("busy=true;setButtons();");assert.equal(e.element("open-diagnostics").disabled,true);assert.equal(e.requests.length,0);
  e.run("busy=false;project.status.read_only=true;setButtons();");assert.equal(e.element("open-diagnostics").disabled,false);
});

test("GET and issue navigation preserve project, unsaved note, accepted candidate, and original coordinate view",async()=>{
  const e=await environment();e.load();e.element("note").value="未确认待办";e.element("note").oninput();
  e.run("view={x:106000000000,y:29000000000,scale:0.002};");const before=e.run("JSON.stringify({project,view,pendingNote,busy})");
  await e.show();assert.equal(e.requests.length,1);assert.equal(e.requests[0].url,"/api/projects/p1/surface-diagnostics");assert.equal(e.requests[0].options.method,"GET");
  e.element("diagnostics-issues").children[0].click();e.element("diagnostics-zoom-in").click();
  e.element("diagnostics-map").onpointerdown({offsetX:20,offsetY:20,pointerId:1});e.element("diagnostics-map").onpointermove({offsetX:30,offsetY:30});
  e.element("diagnostics-map").onpointerup();e.element("close-diagnostics").click();
  assert.equal(e.run("JSON.stringify({project,view,pendingNote,busy})"),before);assert.equal(e.element("note").value,"未确认待办");e.assertEmpty();
});

test("Polygon holes and MultiPolygon members use evenodd fill in the separate canvas",async()=>{
  const e=await environment();e.load();await e.show();
  const fills=e.calls.filter(call=>call.id==="diagnostics-map"&&call.name==="fill");assert.equal(fills.length,4);assert.ok(fills.every(call=>call.args[0]==="evenodd"));
  assert.equal(e.calls.filter(call=>call.id==="diagnostics-map"&&call.name==="closePath").length,8);
  assert.match(e.element("diagnostics-status").textContent,/研究候选未接受.*正式交付仍阻断/);
});

test("loaded geometry shows its local coordinate basis before pointer movement and clears on close",async()=>{
  const e=await environment();e.load();await e.show();
  assert.equal(e.element("diagnostics-coordinates").textContent,"局部米制坐标 · 绝对位置未核验");
  e.element("close-diagnostics").click();assert.equal(e.element("diagnostics-coordinates").textContent,"尚无诊断几何");
});

test("source issue identifiers and untrusted text are rendered as text and selection fits the issue",async()=>{
  const e=await environment();e.load();const value=report('<img src=x onerror="bad()">');await e.show(value);
  const row=e.element("diagnostics-issues").children[0];assert.equal(row.textContent,value.issues[0].title);assert.equal(row.innerHTML,undefined);
  row.click();assert.equal(row.attributes["aria-pressed"],"true");assert.match(e.element("diagnostics-detail").textContent,/original-lane/);
  e.element("diagnostics-map").onpointermove({offsetX:450,offsetY:250});assert.match(e.element("diagnostics-coordinates").textContent,/x 110\.000 m.*y 10\.000 m/);
});

for(const outcome of ["success","error"]){
  for(const change of ["close","project","revision","reset"]){
    test(`late ${outcome} after ${change} cannot show previous geometry`,async()=>{
      const e=await environment();e.load();const old=e.open();
      if(change==="close")e.element("close-diagnostics").click();
      if(change==="project")e.load("p2");
      if(change==="revision")e.run("project={...project,revision:8};applyProject(project);");
      if(change==="reset")e.run("applyProject(project,true);");
      const state=e.element("diagnostics-status").textContent;
      if(outcome==="success")old.request.respond({available:true,report:report("OLD")});else old.request.reject(new Error("OLD error"));
      await old.task;e.assertEmpty();assert.equal(e.element("diagnostics-dialog").open,false);assert.equal(e.element("diagnostics-status").textContent,state);
    });
  }
  test(`older open ${outcome} cannot replace newer response`,async()=>{
    const e=await environment();e.load();const old=e.open();e.element("close-diagnostics").click();const current=e.open();
    // Native dialog close events are queued and may arrive after a reopen.
    e.element("diagnostics-dialog").onclose();current.request.respond({available:true,report:report("CURRENT")});await current.task;
    if(outcome==="success")old.request.respond({available:true,report:report("OLD")});else old.request.reject(new Error("OLD error"));await old.task;
    assert.equal(e.element("diagnostics-issues").children[0].textContent,"CURRENT");
  });
}

test("an unavailable or failed new read clears all previous report geometry and context",async()=>{
  const e=await environment();e.load();await e.show();let pending=e.open();e.assertEmpty();pending.request.respond({available:false,reason:"来源不匹配"});await pending.task;
  e.assertEmpty();assert.equal(e.element("diagnostics-status").textContent,"来源不匹配");
  await e.show();pending=e.open();pending.request.reject(new Error("source drift"));await pending.task;e.assertEmpty();assert.match(e.element("diagnostics-status").textContent,/source drift/);
});

for(const mutate of [
  value=>value.context.frame.unit="degrees",value=>value.context.frame.kind="raw",value=>value.context.frame.origin=[null,0],
  value=>value.context.frame.absolute_crs_status="verified",value=>value.context.candidate_accepted=true,value=>value.context.formal_release_verified=true,
  value=>value.layers[0].geometry.coordinates=[null],value=>value.issues[0].geometry.coordinates[0][0][0]=Infinity,
  value=>value.issues.push({...value.issues[0]})
]){
  test(`invalid report remains empty: ${mutate}`,async()=>{
    const e=await environment();e.load();const value=report();mutate(value);await e.show(value);e.assertEmpty();assert.match(e.element("diagnostics-status").textContent,/未显示几何/);
  });
}

test("diagnostics modal retains Escape precedence over an unconfirmed background note",async()=>{
  const e=await environment();e.load();e.element("note").value="未确认";e.element("note").oninput();const pending=e.open();
  e.listeners.get("keydown")({key:"Escape",preventDefault(){}});assert.equal(e.run("pendingNote"),true);assert.equal(e.element("note").value,"未确认");
  let prevented=false;e.element("diagnostics-dialog").oncancel({preventDefault(){prevented=true;}});assert.equal(prevented,true);assert.equal(e.element("diagnostics-dialog").open,false);
  pending.request.respond({available:true,report:report()});await pending.task;e.assertEmpty();
});
