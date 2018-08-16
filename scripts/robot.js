'use strict';

function score_ranges_from_grid_map(pose, ranges, bearings, gmap){
	const w = gmap.info.width
	const h = gmap.info.height

	var cost = 0

	for(var i=0;i<ranges.length;i++){
		var dist;
		for(dist = 0; dist<100; dist+=gmap.info.resolution){
			var pos =	project_ray(pose, dist, bearings[i])
			pos = descretize(pos, gmap.info.resolution)		
			if(gmap.data[pos.x+pos.y*w]>50)
				break
		}

		if(ranges[i]==Infinity)
			continue

		cost += Math.abs(ranges[i]-dist)
	}
	return cost
}


var toEuler = function(q){
	const x = q.x
	const y = q.y
	const z = q.z
	const w = q.w

  const t0 = 2.0 * (w * x + y * z);
  const t1 = 1.0 - 2.0 * (x * x + y * y);
  const X = Math.atan2(t0, t1);

  var t2 = Math.max(Math.min(2.0 * (w * y - z * x), 1.0), -1.0);
  const Y = Math.asin(t2);

  const t3 = 2.0 * (w * z + x * y);
  const t4 = 1.0 - 2.0 * (y * y + z * z);
  const Z = Math.atan2(t3, t4);

  return {x: X, y: Y, z: Z}
}

function project_ray(pose, range, bearing){
	var th = toEuler(pose.orientation).z
  var pos = {
  	x: pose.position.x + range * Math.cos(th + bearing),
		y: pose.position.y + range * Math.sin(th + bearing)
	}
  return pos
}

function descretize(position, resolution){
	return {
		x: Math.floor(position.x / resolution),
		y: Math.floor(position.y / resolution)
	}
}

function project_laser_to_grid_map(pose, ranges, bearings, map0){
	const w = map0.info.width
	const h = map0.info.height

	if(true || map0.data==undefined){
		map0.data = Array(w*h)
		for(var i=0;i<map0.data.length;i++)
			map0.data[i] = 0.
	}

	for(var i=0;i<ranges.length;i++){
		var pos =	project_ray(pose, ranges[i], bearings[i])

		if(pos.x == Infinity || pos.y ==NaN)
			continue

		// console.log("marked " + pos.x + " , " + pos.y + " at " + map0.info.resolution)

		pos = descretize(pos, map0.info.resolution)		

		map0.data[pos.x+pos.y*w] = 100

	}

	return map0
}

module.exports = {
	project_laser_to_grid_map: project_laser_to_grid_map,
	score_ranges_from_grid_map: score_ranges_from_grid_map,
};