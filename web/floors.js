/* Floor viewing never changes the robot's physical floor or gait. */
let goalFloor = null;
function floorInfo() { return state?.floor_state || {}; }
function floorIdForKey(key) { return Object.entries(floorInfo().floors || {}).find(([,v])=>v.map_key===key)?.[0]; }
function floorKey(id) { return floorInfo().floors?.[id]?.map_key; }
function actualView() { return !!floorInfo().actual_floor && floorKey(floorInfo().actual_floor)===floor; }
function floorLabel(id) { return floorInfo().floors?.[id]?.label || id || '过渡中／待确认'; }
function drawStar(p,label,color='#bc7700') {
  const [x,y]=pixel(p.x,p.y);ctx.save();ctx.translate(x,y);ctx.scale(1/zoom,1/zoom);
  ctx.beginPath();for(let i=0;i<10;i++){const a=-Math.PI/2+i*Math.PI/5,r=i%2?6:14;i?ctx.lineTo(Math.cos(a)*r,Math.sin(a)*r):ctx.moveTo(Math.cos(a)*r,Math.sin(a)*r)}
  ctx.closePath();ctx.fillStyle=color;ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
  ctx.fillStyle='#754800';ctx.font='12px sans-serif';ctx.fillText(label,18,-12);ctx.restore();
}
function drawConnections() {
  const fs=floorInfo(),view=floorIdForKey(floor);
  for(const segment of previewPath?.floor_reference_paths||[]){
    if(segment.floor!==floor)continue;
    ctx.save();ctx.setLineDash([5/zoom,4/zoom]);line(segment.points,'#8745a3',3);ctx.restore();
  }
  for(const c of fs.connections||[]) {
    if(c.from_floor===view)drawStar(c.entry,c.label);
  }
  for(const c of previewPath?.mission?.connections||[]) {
    if(view!==c.from_floor&&view!==c.to_floor)continue;
    const endpoint=c.preview_exit||c.exit;
    if(endpoint){
      ctx.save();ctx.setLineDash([7/zoom,5/zoom]);line([[c.entry.x,c.entry.y],[endpoint.x,endpoint.y]],'#bc7700',2);ctx.restore();
      if(view===c.to_floor)drawStar(endpoint,c.preview_exit?'楼梯预备出口':'楼梯出口','#6d6aaf');
    }
  }
}
function renderFloors() {
  const fs=floorInfo(),view=floorIdForKey(floor),task=navigationStatus?.task;
  const floorSelect=$('floor'),signature=JSON.stringify(fs.floors||{});
  if(floorSelect.floorSignature!==signature){
    floorSelect.replaceChildren(...Object.entries(fs.floors||{}).map(([id,v])=>{const o=document.createElement('option');o.value=v.map_key;o.textContent=v.label||id;return o}));
    floorSelect.value=floor;floorSelect.floorSignature=signature;
  }
  $('floorState').textContent=`查看：${floorLabel(view)} ｜ 实际：${floorLabel(fs.actual_floor)} ｜ 目标：${(navigationStatus?.active?floorLabel(task?.target_floor):previewPath?.target_floor||fs.target_floor?floorLabel(previewPath?.target_floor||fs.target_floor):'未设置')} ｜ 阶段：${fs.phase||'未知'}`;
  $('coordinateType').textContent=actualView()?'当前楼层地图坐标':`虚拟投影坐标（机器人实际：${floorLabel(fs.actual_floor)}，切换查看不切换实际楼层）`;
  const select=$('stairConnection'),keep=select.value;
  select.replaceChildren(...(fs.connections||[]).filter(c=>c.enabled).map(c=>{const o=document.createElement('option');o.value=c.id;o.textContent=`${c.label}：${c.from_floor} → ${c.to_floor}`;return o}));
  if([...select.options].some(o=>o.value===keep))select.value=keep;
  const c=(fs.connections||[]).find(c=>c.id===select.value);
  $('planStair').disabled=!online||planning||!!navigationStatus?.active||!c||c.from_floor!==fs.actual_floor;
  $('selectStair').disabled=planning||!c;
  $('manualInitialPose').disabled=!!navigationStatus?.active||!online;
  if(online&&state?.pose_current&&!actualView())$('notice').textContent='当前位置显示为所选楼层上的虚拟投影；实际楼层不会因查看地图而改变。';
  if(fs.actual_floor===null)$('notice').textContent=`楼层过渡／待确认：${fs.phase}。可查看各层虚拟投影，暂不允许普通平地导航。`;
}
async function planSelectedGoal(connection=null) {
  if(planning)return;
  try {
    if(!online||!state?.navigation_ready)throw Error('定位未就绪；可停稳后使用手动初值');
    goal=connection?{...connection.entry}:readGoal();goalFloor=connection?floorKey(connection.from_floor):floor;
    clearPreview();const revision=goalRevision;planning=true;pickMode=false;
    $('pickGoal').classList.remove('active');canvas.style.cursor='grab';
    for(const id of ['planGoal','pickGoal','clearGoal','goalX','goalY','goalYaw'])$(id).disabled=true;
    const body={...goal,scene,floor};
    if(connection)Object.assign(body,{floor:floorKey(connection.to_floor),connection_id:connection.id,transition_only:true});
    message('正在规划楼层路线和首段平地路径…');
    const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),55000);
    try {
      const r=await fetch('/preview-plan',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:controller.signal});
      const result=await r.json();if(revision!==goalRevision)return;
      if(!r.ok||!result.ok){$('failureDetails').hidden=false;$('failureData').textContent=JSON.stringify(result,null,2);throw Error(result.error||'规划失败')}
      previewPath=result;
      if(result.mission&&Number($('maxRuntime').value)===10)$('maxRuntime').value=180;
      message(result.mission?`${result.mission.connections.map(c=>c.from_floor+' → '+c.to_floor).join(' → ')}；首段 ${result.length_m.toFixed(2)} m。${result.note}`:`路线 ${result.length_m.toFixed(2)} m。普通导航到点，不触发步态切换。`);
    } finally {clearTimeout(timer)}
  } catch(e) {previewPath=null;message(e.name==='AbortError'?'规划超时':e.message)}
  finally {planning=false;for(const id of ['planGoal','pickGoal','clearGoal','goalX','goalY','goalYaw'])$(id).disabled=false;renderFloors();draw()}
}
function setupFloors() {
  $('selectStair').onclick=()=>{
    const c=floorInfo().connections?.find(c=>c.id===$('stairConnection').value);if(!c)return;
    floor=floorKey(c.from_floor);$('floor').value=floor;if(state){meta=state.maps[floor];load()}
    goal={...c.entry};goalFloor=floor;clearPreview();writeGoal();
    message('已选择星标预备点。点击“规划预览”仅导航到点；点击“规划经此楼梯上楼”才包含 Stair。');draw();
  };
  $('planStair').onclick=()=>planSelectedGoal(floorInfo().connections?.find(c=>c.id===$('stairConnection').value));
  $('manualInitialPose').onclick=async()=>{
    try {
      const p=readGoal(),text=$('initialZ').value.trim();if(text==='')throw Error('请输入当前机身在地图中的 Z 高度；不能把一层高度直接用于二层');
      const z=Number(text);if(!Number.isFinite(z))throw Error('Z 高度无效');
      if(!confirm(`将 ${floorLabel(floorIdForKey(floor))} 的 (${p.x}, ${p.y}, ${z})，朝向 ${p.yaw.toFixed(4)} rad 设为当前位置？请保持停稳。此操作不移动机器人，已有路线将失效。`))return;
      $('manualInitialPose').disabled=true;
      const response=await fetch('/localization/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...p,z,scene,floor,confirmed:true})});
      const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error||'初值设置失败');
      clearPreview();goal=null;message(result.message);
    }catch(e){message(e.message)}finally{$('manualInitialPose').disabled=false}
  };
  $('useCurrentInitialZ').onclick=()=>{
    if(!state?.pose_current||!actualView()){message('当前查看楼层不是已确认实际楼层，请手动填写高度。');return}
    $('initialZ').value=state.pose.z.toFixed(3);
  };
  const down=canvas.onpointerdown;
  canvas.onpointerdown=e=>{
    if(!pickMode&&!planning){const r=canvas.getBoundingClientRect();
      const c=(floorInfo().connections||[]).find(c=>c.from_floor===floorIdForKey(floor)&&(()=>{const q=pixel(c.entry.x,c.entry.y);return Math.hypot(q[0]*zoom+px-(e.clientX-r.left),q[1]*zoom+py-(e.clientY-r.top))<17})());
      if(c){$('stairConnection').value=c.id;$('selectStair').onclick();return}}
    if(pickMode)goalFloor=floor;down(e);
  };
}
