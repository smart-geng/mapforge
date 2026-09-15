import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source=await readFile(new URL('../mapforge/repair_web/static/app.js',import.meta.url),'utf8');
const {toScreen,toWorld,dragValue,zoomAt,resizeView,pickLines,LatestPreview}=
  await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-10,`${a} != ${b}`);

test('off-centre grabs use relative movement, no initial jump',()=>{
  near(dragValue(.25,[121,99],[121,99],[0,1],8),.25);
  near(dragValue(.25,[121,99],[121,91],[0,1],8),1.25);
  near(dragValue(.25,[101,109],[101,101],[0,1],8),1.25);
});
test('normal projection ignores tangential motion',()=>{
  near(dragValue(0,[0,0],[6,-8],[.8,-.6],10),0);
});
test('zoom preserves the point under cursor',()=>{
  const v={x:320,y:90,k:3},p=[120,180],w=toWorld(p,v),z=zoomAt(v,p,2);
  const q=toScreen(w,z);near(q[0],p[0]);near(q[1],p[1]);
});
test('resizing preserves world centre',()=>{
  const v={x:400,y:250,k:8},old=[1100,800],size=[540,720];
  assert.deepEqual(toWorld([550,400],v),toWorld([270,360],resizeView(v,old,size)));
});
test('picking tolerance is in screen pixels and returns semantic IDs',()=>{
  const geometry={a:{lines:[[[0,0],[100,0]]]},b:{lines:[[[0,2],[100,2]]]}};
  for(const k of [1,10,80]){
    const v={x:0,y:0,k};
    assert.ok(pickLines([50*k,9],geometry,v).some(h=>h.id==='a'));
    assert.equal(pickLines([50*k,11],geometry,v).some(h=>h.id==='a'),false);
  }
  assert.deepEqual(pickLines([50,-1],geometry,{x:0,y:0,k:1}).map(h=>h.id),['a','b']);
});
const tick=()=>new Promise(resolve=>setImmediate(resolve));
test('preview requests are serial, latest wins, intermediates coalesce',async()=>{
  const calls=[],resolvers=[],results=[];
  const q=new LatestPreview(p=>{calls.push(p);return new Promise(r=>resolvers.push(r));},p=>results.push(p),e=>{throw e;});
  q.submit('one');q.submit('two');q.submit('three');
  assert.deepEqual(calls,['one']);resolvers.shift()('stale');await tick();
  assert.deepEqual(calls,['one','three']);assert.deepEqual(results,[]);
  resolvers.shift()('latest');await tick();assert.deepEqual(results,['latest']);assert.equal(q.pending,false);
});
test('Escape-style cancel ignores late reply without cancelling next request',async()=>{
  const resolvers=[],results=[];
  const q=new LatestPreview(()=>new Promise(r=>resolvers.push(r)),p=>results.push(p),e=>{throw e;});
  q.submit('old');q.cancel();q.submit('new');resolvers.shift()('cancelled');await tick();
  assert.deepEqual(results,[]);resolvers.shift()('new');await tick();assert.deepEqual(results,['new']);
});
test('rejected preview does not get reported as a result',async()=>{
  const errors=[],results=[];
  const q=new LatestPreview(async()=>{throw new Error('negative width');},p=>results.push(p),e=>errors.push(e.message));
  q.submit({});await tick();assert.deepEqual(results,[]);assert.deepEqual(errors,['negative width']);assert.equal(q.pending,false);
});
