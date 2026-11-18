const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const elements=new Map(),posts=[];
const ctx=new Proxy({}, {get:()=>()=>{}});
function el(id){if(!elements.has(id))elements.set(id,{value:'',checked:true,options:[],setAttribute(){},replaceChildren(...items){this.options=items;this.value=this.value||items[0]?.value||''},textContent:'',style:{},classList:{toggle(){},remove(){}},getContext:()=>ctx,getBoundingClientRect:()=>({width:800,height:570,left:0,top:0}),addEventListener(){},removeAttribute(){},showModal(){this.open=true},close(){this.open=false},focus(){}});return elements.get(id)}
const sandbox={document:{getElementById:el,createElement:()=>({value:'',textContent:''}),hidden:false},location:{search:'?scene=old'},URLSearchParams,performance,AbortController,setTimeout:()=>1,clearTimeout(){},window:{devicePixelRatio:1},Image:function(){},console};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(require('path').join(__dirname,'../web/floors.js'),'utf8'),sandbox);
const html=fs.readFileSync(require('path').join(__dirname,'../web/index.html'),'utf8');
vm.runInContext(html.split('<script>')[1].split('</script>')[0].replace('poll();pollCostmap();','').replace(/\npollNavigation\(\);/,''),sandbox);
const run=code=>vm.runInContext(code,sandbox);
const init=()=>run("state={navigation_ready:true};online=true;navigationOnline=true;navigationStatus={active:false,task:null};previewPath={plan_id:'plan1',goal:{x:1,y:2,yaw:-.5},length_m:.3};");
(async()=>{
 init();el('maxRuntime').value='12';
 sandbox.fetch=async(url,args)=>{posts.push({url,body:JSON.parse(args.body)});return {ok:true,json:async()=>({ok:true,active:true,task:{task_id:'task1',state:'PREPARING',max_runtime:12,reason:'preparing'},logs:[]})}};
 el('navigateGoal').onclick();assert.equal(el('navConfirm').open,true);assert.equal(posts.length,0);assert.match(el('navConfirmText').textContent,/-0.5000 rad/);
 el('cancelNavConfirm').onclick();assert.equal(el('navConfirm').open,false);assert.equal(posts.length,0);
 el('navigateGoal').onclick();await el('confirmNavigation').onclick();assert.equal(posts.length,1);assert.deepEqual(posts[0].body,{plan_id:'plan1',max_runtime:12,confirmed:true});assert.equal(el('navigateGoal').disabled,true);
 await el('confirmNavigation').onclick();assert.equal(posts.length,1);
 await el('stopNavigation').onclick();assert.equal(posts[1].url,'/navigation/stop');assert.deepEqual(posts[1].body,{task_id:'task1'});
 init();el('navigateGoal').onclick();run('goalRevision++');await el('confirmNavigation').onclick();assert.equal(posts.length,2);assert.match(el('planMessage').textContent,/已改变/);
 el('maxRuntime').value='0';el('navigateGoal').onclick();assert.equal(el('navConfirm').open,false);assert.match(el('planMessage').textContent,/大于 0/);
 run('navigationOnline=false;renderNavigation()');assert.equal(el('navigateGoal').disabled,true);
 run("state={map_id:'m'};scene='old';online=true;navigationOnline=true;navigationStatus={active:true,task:{task_id:'a'},telemetry:{task_id:'a',fresh:true,route:{task_id:'a',route_id:'a:2',map_id:'m',scene:'old',status:'executing',points:[[0,0],[1,0]]}}};");
 assert.equal(run('executionView().live'),true);
 run('navigationStatus.telemetry.fresh=false');assert.equal(run('executionView().live'),false);
 run("navigationStatus.telemetry.fresh=true;navigationStatus.telemetry.route.task_id='b'");assert.equal(run('executionView().same'),false);
 run("state={floor_state:{actual_floor:'floor1',floors:{floor1:{map_key:'lower'},floor2:{map_key:'upper'}}}};floor='upper'");assert.equal(run('actualView()'),false);assert.equal(run("floorIdForKey('upper')"),'floor2');
 console.log('PASS: confirmation, cancel, runtime/radians, duplicate prevention, stale goal, stop task binding, disconnect.');
})().catch(e=>{console.error(e);process.exitCode=1});
