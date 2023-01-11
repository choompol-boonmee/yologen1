use std::fs::File;
use std::io::prelude::*;
use std::path::Path;
use std::fs;
use rand::prelude::*;
use std::process::Command;
use std::io::{self, BufRead};
use image::{Rgba};
use imageproc::drawing::draw_hollow_rect_mut;
//use imageproc::drawing::draw_line_segment_mut;
use imageproc::rect::Rect;
use std::str::FromStr;
use std::collections::HashMap;

fn cpt() -> String {
    let path = std::env::current_dir().unwrap();
    let path = path.to_str().unwrap();
    let path = path.replace("\\","\\\\");
    String::from(path)
}

fn main() {
	let a1 = std::env::args().nth(1).unwrap_or("none".to_string());
	match a1.as_str() {
	"train" => train(),
	_ => println!("no task"),
	}
}

/*
cargo run train work work/set1 9 12 640
*/

fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where P: AsRef<Path>, {
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}

fn train() {

	let md = std::env::args().nth(2).unwrap_or("none".to_string());
	let rt = std::env::args().nth(3).unwrap_or("none".to_string());
    let md = md.as_str();
    let rt = rt.as_str();
    if rt=="none" || md=="none" { return; }

	let row = std::env::args().nth(4).unwrap_or("none".to_string());
    let row = row.to_string().parse::<i32>().unwrap();

	let col = std::env::args().nth(5).unwrap_or("none".to_string());
    let col = col.to_string().parse::<i32>().unwrap();

	let res = std::env::args().nth(6).unwrap_or("none".to_string());
    let res = res.to_string().parse::<i32>().unwrap();

    println!("row: {:?}", &row);
    println!("col: {:?}", &col);
    println!("res: {:?}", &res);

    let mut fscene = String::new();
    let mut modlist = vec![];

    let scene = fs::read_dir(format!("{}/scene",&md));
    let mut scid = 0;
    if let Ok(scene) = scene {
        for path in scene {
            let s = path.unwrap().path().into_os_string()
                .into_string().unwrap();
            let s = s.replace("\\","/");
            let p1 = s.rfind("scene-");
            let p2 = s.rfind(".blend");
            if s.ends_with(".blend") {
                if let (Some(p1),Some(p2)) = (p1,p2) {
                    let p3 = &s[p1+6..p2];
                    let d1 = p3.to_string().parse::<i32>();
                    if let Ok(d1)=d1 {
                        if d1>scid {
                            scid = d1;
                            fscene = s.to_string();
                        }
                    }
                }
            }
        }
    }
    if scid==0 { return }
    println!("scene file: {}", fscene);

    let dmod = format!("{}/model",&md);
    let models = fs::read_dir(&dmod);
    if let Ok(models) = models {
        for path in models {

            let s = path.unwrap().path().into_os_string()
                .into_string().unwrap();
            let s = s.replace("\\","/");
            let p1 = s.rfind('/');
            if let Some(p1) = p1 {
                let d1 = &s[p1+1..];
                let indrs = fs::read_dir(&s);
                if let Ok(indrs) = indrs {
                    let mut lsb = String::new();
                    let mut mof = String::new();
                    let mut max = 0;
                    for f_indr in indrs {
                        let f1 = f_indr.unwrap().path()
                            .into_os_string()
                            .into_string().unwrap();
                        let f1 = f1.replace("\\","/");
                        let p2 = f1.rfind('/').unwrap();
                        let p3 = f1.ends_with(".blend");
                        if p3 {
                            let d2 = &f1[p2+1..f1.len()-6];
                            if d2.starts_with(d1) {
                                let d3 = &d2[d1.len()+1..];
                                let d4 = d3.to_string().parse::<i32>();
                                if let Ok(d4)=d4 {
                                    if d4>max {
                                        max = d4;
                                        lsb = f1.to_string();
                                        mof = d1.to_string();
                                    }
                                }
                            }
                        }
                    }
                    if lsb.len()>0 {
                        modlist.push((lsb.clone(),mof.clone()));
                    }
                }
            }
        }
    }
    if modlist.len()==0 {
        return;
    }

    for s in &modlist {
        println!("model {} {}", s.0, s.1);
    }

	let work = format!("{rt}/work/");
	fs::create_dir_all(&work).unwrap();

    use std::fmt::Write;
    let mut wks = String::new();
	writeln!(&mut wks, 
r###"
import bpy

f = open("{}/work/point.txt", "w")

p0 = bpy.data.objects['point0']
txt = "%f %f %f\n" % (p0.location.x, p0.location.y, p0.location.z)
f.write(txt)

p1 = bpy.data.objects['point1']
txt = "%f %f %f\n" % (p1.location.x, p1.location.y, p1.location.z)
f.write(txt)

p2 = bpy.data.objects['point2']
txt = "%f %f %f\n" % (p2.location.x, p2.location.y, p2.location.z)
f.write(txt)

p3 = bpy.data.objects['point3']
txt = "%f %f %f\n" % (p3.location.x, p3.location.y, p3.location.z)
f.write(txt)

f.close()

"###, rt
    ).unwrap();

	let wkfn = format!("{rt}/work/point.py");
	let wkpt = Path::new(&wkfn);
    if let Ok(mut wkf)=File::create(&wkpt) {
        wkf.write_all(wks.as_bytes()).expect("");
    }

	let cmd = ["blender","-b", &fscene, "-P", &wkfn];
	println!("  {:?}", cmd);
	let mut child = Command::new(cmd[0]).args(&cmd[1..]).spawn().unwrap();
	let _result = child.wait().unwrap();

    let mut objs = vec![];
	let pnfn = format!("{rt}/work/point.txt");
	if let Ok(lines) = read_lines(&pnfn) {
		for line in lines {
			if let Ok(line) = line {
				let dds : Vec<f64> = line.split(" ")
                    .map(|x| x.parse().expect("No float")).collect();
                objs.push(dds.clone());
                println!("{:?}", dds);
            }
        }
    }

    println!("{:?}", objs.len());

    // ============= READ MODEL ALL
    let mut wks = String::new();
    writeln!(&mut wks, "import bpy").unwrap();
    writeln!(&mut wks, "import os").unwrap();
	let mut wds = vec![];
    let mut xx0 = 0.0;
    let cpt0 = cpt();
    let cpt0 = cpt0.replace("\\\\","/");
    for s in &modlist {
        writeln!(&mut wks, "").expect("");
        writeln!(&mut wks, "").expect("");
        writeln!(&mut wks, "file = '{}/{}'", cpt0, s.0).expect("");
        writeln!(&mut wks, "path = 'Object'").expect("");
        writeln!(&mut wks, "name = '{}'", s.1).expect("");
        writeln!(&mut wks, r###"
bpy.ops.wm.append(
  filepath = os.path.join(file, path, name),
  directory= os.path.join(file, path),
  filename=name)
bpy.ops.transform.translate(value=({},-10,0))
        "###, xx0).expect("");
        xx0 += 0.5;
        wds.push(s.1.to_string());
//        let s2: &'static str = s.1.clone().as_str();
//        wds.push(s2);
    }
    let scene1 = format!("{}/{}/work/scene.blend", cpt0, rt);
    let scene1 = scene1.replace("\\","/");
//    writeln!(&mut wks, "file = '{}/{}/work/scene.blend'", cpt0, rt).expect("");
    writeln!(&mut wks, "file = '{}'", scene1).expect("");
    writeln!(&mut wks, "bpy.ops.wm.save_as_mainfile(filepath=file)").expect("");

	let wkfn = format!("{rt}/work/models.py");
	let wkpt = Path::new(&wkfn);
    if let Ok(mut wkf)=File::create(&wkpt) {
        wkf.write_all(wks.as_bytes()).expect("");
    }

    let ax = objs[1][0];
    let ay = objs[1][1];
    let az = objs[1][2];

    let bx = objs[2][0];
    let by = objs[2][1];

    let cx = objs[3][0];
    let cy = objs[3][1];

	let cmd = ["blender","-b", &fscene, "-P", &wkfn];
	println!("  {:?}", cmd);
	let mut child = Command::new(cmd[0]).args(&cmd[1..]).spawn().unwrap();
	let _result = child.wait().unwrap();

	let a = bx - ax;
	let b = by - ay;
	let c = cx - ax;
	let d = cy - ay;
	println!("a:{} b:{} c:{} d:{}", a, b, c, d);

	let d1x = cx + a;
	let d1y = cy + b;
	println!("d1x:{} d1y:{}", d1x, d1y);

	let imtr = format!("{rt}/train/images/train");
	let imva = format!("{rt}/train/images/val");
	let imte = format!("{rt}/train/images/test");
	let lbtr = format!("{rt}/train/labels/train");
	let lbva = format!("{rt}/train/labels/val");
	let lbte = format!("{rt}/train/labels/test");
	let toch = format!("{rt}/check/");
	let pys = format!("{rt}/pys");

	let mut cnt = 0;
    let paths = fs::read_dir(&toch);
    if let Ok(paths) = paths {
        for path in paths {
            let s = path.unwrap().path().into_os_string()
                .into_string().unwrap();
            let p1 = s.rfind('/');
            let p2 = s.rfind('.');
            if let Some(p1) = p1 {
                if let Some(p2) = p2 {
                    let nn = &s[p1+1..p2];
                    if p2-p1>1 {
                        let n: i32 = FromStr::from_str(nn).unwrap();
                        if n>cnt {
                            cnt = n;
                        }
                    }
                }
            }
        }
    }
//    if true { return; }

	fs::create_dir_all(&imtr).unwrap();
	fs::create_dir_all(&imva).unwrap();
	fs::create_dir_all(&imte).unwrap();
	fs::create_dir_all(&lbtr).unwrap();
	fs::create_dir_all(&lbva).unwrap();
	fs::create_dir_all(&lbte).unwrap();
	fs::create_dir_all(&toch).unwrap();
	fs::create_dir_all(&pys).unwrap();

	println!("======================= {} {}", row, col);
//    use std::fmt::Write;
    let mut yolc = String::new();
	writeln!(&mut yolc, 
r###"path: "./train"
train: images/train
val: images/val
test: images/test"###).unwrap();
    writeln!(&mut yolc, "nc: {}", wds.len()).unwrap();
    writeln!(&mut yolc, "names: [").unwrap();
    for w in &wds {
        writeln!(&mut yolc, r###"  "{}","###, &w).unwrap();
    }
    writeln!(&mut yolc, "]").unwrap();

	let yolm = format!("{rt}/train.yaml");
	let yolp = Path::new(&yolm);
    if let Ok(mut yolf)=File::create(&yolp) {
        yolf.write_all(yolc.as_bytes()).expect("");
    }

    let all = row * col;
    let cn1 : i32 = all / 3;

//	for num in cn1..=all {
	for num in 1..=cn1 {
		for va in [0.0,5.0,10.0] {
			for ha in [-15.0,-10.0,-5.0,0.0,5.0,10.0,15.0] {
				println!("va {} ha {}", va, ha);
				for _i0 in 0..3 {
					cnt += 1;
					mod_mis_create(cnt, num, &fscene, va, ha
						, (&pys,&imtr,&imva,&imte,&lbtr,&lbva,&lbte,&toch)
						, (&ax,&ay,&az,&a,&b,&c,&d, &row, &col)
                        , &modlist, &res);
				}
			}
		}
	}
}

//============ MOD_MIS_CREATE =============
#[allow(dead_code)]
fn mod_mis_create(cnt:i32, mis:i32, bld:&str, va:f32, ha:f32
	, (pys,imtr,imva,imte,lbtr,lbva,lbte,toch)
       : (&str,&str,&str,&str,&str,&str,&str,&str)
	, (ax,ay,az,a,b,c,d,row,col)
       : (&f64,&f64,&f64, &f64,&f64,&f64,&f64, &i32,&i32)
	, modlist: &Vec<(String,String)>, res: &i32) {

	let mut todo:Vec<i32> = vec![];
	let irow = row.to_owned() as i32;
	let icol = col.to_owned() as i32;
	for i in 0..(irow*icol) {
		todo.push(i);
	}
	use std::fmt::Write;
	let mut s = String::new();
	writeln!(&mut s, "import bpy").unwrap();
	writeln!(&mut s, "import os").unwrap();
	writeln!(&mut s, "import bpy_extras").unwrap();
	writeln!(&mut s, "").unwrap();

	let va = - 3.1416 * va / 180.0;
	let ha = 3.1416 * ha / 180.0;
    writeln!(&mut s, "bpy.data.scenes['Scene'].render.resolution_x={}",res).unwrap();
    writeln!(&mut s, "bpy.data.scenes['Scene'].render.resolution_y={}",res).unwrap();
	writeln!(&mut s, "bpy.ops.object.select_all(action='DESELECT')").unwrap();
	writeln!(&mut s, "bpy.data.objects['Focus'].select_set(True)").unwrap();
	writeln!(&mut s, "ov=bpy.context.copy()").unwrap();
	writeln!(&mut s, "ov['area']=[a for a in bpy.context.screen.areas if a.type=='VIEW_3D'][0]").unwrap();
	writeln!(&mut s, "bpy.ops.transform.rotate(ov, value={}, orient_axis='X')", va).unwrap();
	writeln!(&mut s, "bpy.ops.transform.rotate(ov, value={}, orient_axis='Z')", ha).unwrap();

    // ==== LOAD MODEL

	let _rng = rand::thread_rng();
	let mut ccc:i32 = 0;

	let mut rng = rand::thread_rng();
    let max = todo.len();
	for _i in 0..mis {
		let ln = todo.len() as f32;
		let rn: f32 = rng.gen();
		let n1 = (rn * ln) as usize;
		todo.remove(n1);
	}
    let cur = todo.len();
	println!(">>>>>>>>>>> from {} to {}", max, cur);

	let mut bx:Vec<(usize, f64,f64,f64)> = vec![];
	for rr in 0..irow {
		for cc in 0..icol {

			let mut fnd = 0;
			for ii in &todo {
				
				if ii==&ccc {
					fnd += 1;
					break;
				}
			}
			ccc += 1;
			if fnd==0 {
				continue;
			}

			let rf = rr as f64 / (row.clone() as f64-1.0);
			let cf = cc as f64 / (col.clone() as f64-1.0);
			let rn: f32 = rng.gen();
            let gc = modlist.len() as f32;
			let pc = rn * gc;
			let pc = pc as usize;
		    let an: f64 = rng.gen();
            let an = an * 0.2;
            let an = an - 0.1;

			bx.push((pc, rf, cf, an));

		}
	}

    let mut hpc = HashMap::new();
    let cpt0 = cpt();
    let cpt0 = cpt0.replace("\\\\","/");
    let mut xx0 = 0.0;
    for (pc, _rf, _cf, _an) in &bx {
        if let None = hpc.get(pc) {
            hpc.insert(pc, modlist[*pc].1.clone());
            println!("MAP_______  {} => {}", pc, modlist[*pc].1.clone());

            writeln!(&mut s, "file = '{}/{}'", cpt0, &modlist[*pc].0).expect("");
            writeln!(&mut s, "path = 'Object'").expect("");
            writeln!(&mut s, "name = '{}'", &modlist[*pc].1).expect("");
            writeln!(&mut s, r###"
bpy.ops.wm.append(
  filepath = os.path.join(file, path, name),
  directory= os.path.join(file, path),
  filename=name)
bpy.ops.transform.translate(value=({},-10,0))
            "###, xx0).expect("");
            xx0 += 0.5;
        }
    }

    for (pc,rf,cf,an) in &bx {
		let dx = ax + rf * a + cf * c;
		let dy = ay + rf * b + cf * d;
		let co = format!("{},{},{}", dx, dy, az);

        let obj0 = modlist[*pc].1.as_str();
		writeln!(&mut s, "bpy.ops.object.select_all(action='DESELECT')").unwrap();
		writeln!(&mut s, "bpy.data.objects['{}'].select_set(True)", obj0).unwrap();

		writeln!(&mut s, "bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={{'linked':False, 'mode':'TRANSLATION'}})").unwrap();
		writeln!(&mut s, "for obj in bpy.context.selected_objects: obj.location = ({})", co).unwrap();
        writeln!(&mut s, "bpy.ops.transform.rotate(value={}, orient_axis='Z')", an).unwrap();
		writeln!(&mut s, "bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)").unwrap();
	
    }

	let mut sf = format!("{}/{}.jpg", imtr, cnt);
	let mut tx = format!("{}/{}.txt", lbtr, cnt);
	if cnt%10==9 {
		sf = format!("{}/{}.jpg", imva, cnt);
		tx = format!("{}/{}.txt", lbva, cnt);
	} else if cnt%10==0 {
		sf = format!("{}/{}.jpg", imte, cnt);
		tx = format!("{}/{}.txt", lbte, cnt);
	}

	let chim = format!("{}/{}.jpg", toch, cnt);
	let chtx = format!("{}/{}.txt", toch, cnt);

	writeln!(&mut s, "bpy.context.scene.render.filepath = '{}/{}'",cpt(),sf).unwrap();
	writeln!(&mut s, "bpy.ops.render.render(write_still=True)").unwrap();
	let mut ss = String::new();
	for ii in 0..modlist.len() {
		let ob = &modlist[ii].1;
		println!(">>> {}", ob);
		write!(&mut ss, "\n    if o.name.find('{}')>=0 and len(o.name)>{} : ob={}", ob, ob.len(), ii).unwrap();
	}
	writeln!(&mut s, r###"
scn = bpy.context.scene
cam = bpy.data.objects['Camera']
f = open("{}", "w")
cn = 0
for o in bpy.data.objects:
    ob = -1;{}
    if ob<0 : continue

    lo = o.location
    co = bpy_extras.object_utils.world_to_camera_view(scn, cam, lo)
    co.y = 1.0 - co.y

    lf=-1; rg=-1; tp=-1; bt=-1;
    for v in o.data.vertices:
        co = lo + v.co
        co = bpy_extras.object_utils.world_to_camera_view(scn, cam, co)
        co.y = 1.0 - co.y
        if lf<0 or co.x<lf : lf = co.x
        if rg<0 or co.x>rg : rg = co.x
        if tp<0 or co.y<tp : tp = co.y
        if bt<0 or co.y>bt : bt = co.y
    ox = (lf+rg)/2
    oy = (tp+bt)/2
    wd = rg-lf
    hg = bt-tp
    txt = "%d %f %f %f %f\n" % (ob,ox,oy,wd,hg)
    f.write(txt)

f.close()
"###,tx,ss).unwrap();

	let pyss = format!("{}/pys-{}.py", pys, cnt);
	println!("PYS: {}", pyss);
	let path = Path::new(&pyss);
	let mut file = match File::create(&path) {
		Err(why) => panic!("couldn't create: {}", why),
		Ok(file) => file,
	};
	match file.write_all(s.as_bytes()) {
		Err(why) => panic!("couldn't write to {}", why),
		Ok(a) => a,
	}

	let cmd = ["blender","-b",bld, "-P", &pyss];
	println!("  {:?}", cmd);

	let mut child = Command::new(cmd[0]).args(&cmd[1..]).spawn().unwrap();
	let _result = child.wait().unwrap();

	println!(" png: {}", &sf);

	let mut tt = String::new();
	if let Ok(lines) = read_lines(&tx) {
		let mut img = image::open(&sf).unwrap();
		for line in lines {
			if let Ok(line) = line {
				let dds : Vec<f32> = line.split(" ").map(|x| x.parse().expect("No float")).collect();
				let x = dds[1];
				let y = dds[2];
				let w = dds[3];
				let h = dds[4];
				let a = *res as f32;
						
				let xc:i32 = (x * a) as i32;
				let yc:i32 = (y * a) as i32;
						
				let x1:i32 = ((x-w/2.0) * a) as i32;
				let y1:i32 = ((y-h/2.0) * a) as i32;
				let ww:u32 = (w * a) as u32;
				let hh:u32 = (h * a) as u32;

                if xc>3 && yc>3 && x1>0 && y1>0 {
	   				draw_hollow_rect_mut(&mut img, Rect::at(xc-3, yc-3).of_size(7, 7)
                        , Rgba([255u8, 0u8, 0u8, 255u8]));
						
	   				draw_hollow_rect_mut(&mut img, Rect::at(x1, y1).of_size(ww, hh)
                        , Rgba([255u8, 255u8, 255u8, 255u8]));

                }
				writeln!(&mut tt, "{}", &line).unwrap();
			}
		}
		img.save(chim).unwrap();
				
	}
	let path = Path::new(&chtx);
    if let Ok(mut file) = File::create(&path) {
        file.write_all(tt.as_bytes()).expect("");
    }
}

/*
//============ MOD_NUM_CREATE =============
#[allow(dead_code)]
fn mod_num_create(cnt:i32, num:i32, bld:&str, va:f32, ha:f32
	, (pys,imtr,imva,imte,lbtr,lbva,lbte,toch)
      :(&str,&str,&str,&str,&str,&str,&str,&str)
	, (ax,ay,az,a,b,c,d,row,col)
      :(&f64,&f64,&f64, &f64,&f64,&f64,&f64, &i32,&i32)
	, _obj: &Vec<String>, modlist: &Vec<(String,String)>) {

    println!("================ num {}", &num);

	let _irow = row.to_owned() as i32;
	let _icol = col.to_owned() as i32;
	use std::fmt::Write;
	let mut s = String::new();
	writeln!(&mut s, "import bpy").unwrap();
	writeln!(&mut s, "import bpy_extras").unwrap();
	writeln!(&mut s, "import os").unwrap();
	writeln!(&mut s, "").unwrap();
	let mut rng = rand::thread_rng();
	writeln!(&mut s, "bpy.ops.object.select_all(action='DESELECT')").unwrap();
	writeln!(&mut s, "bpy.data.objects['Focus'].select_set(True)").unwrap();
	println!(">>>>>> ===== ROT2 {} {}", va, ha);

	let va = - 3.1416 * va / 180.0;
	let ha = 3.1416 * ha / 180.0;
println!("mod num ..0.1");
	writeln!(&mut s, "ov=bpy.context.copy()").unwrap();
	writeln!(&mut s, "ov['area']=[a for a in bpy.context.screen.areas if a.type=='VIEW_3D'][0]").unwrap();

	writeln!(&mut s, "bpy.ops.transform.rotate(ov, value={}, orient_axis='X')", va).unwrap();
println!("mod num ..0.2");
	writeln!(&mut s, "bpy.ops.transform.rotate(ov, value={}, orient_axis='Z')", ha).unwrap();

	let mut bx:Vec<(usize, f64,f64,f64)> = vec![];

println!("mod num ..1");
	for _n in 0..num {

        let gc = modlist.len() as f32;
//        let gc = obj.len() as f32;
		let rn: f32 = rng.gen();
		let pc = rn * gc;
		let pc = pc as usize;
		let an: f64 = rng.gen();
        let an = an * 0.2;
        let an = an - 0.1;

		'outer: loop {
			let rf: f64 = rng.gen();
			let cf: f64 = rng.gen();

			for tp in &bx {
				let dr = rf - tp.1;
				let dc = cf - tp.2;
				let ds = f64::sqrt(dr*dr+dc*dc);
				if ds<0.30 {
					continue 'outer;
				}
			}

			bx.push((pc, rf, cf, an));
			break;
		}
	}

println!("mod num ..2");
    let mut hpc = HashMap::new();
    let cpt0 = cpt();
    let cpt0 = cpt0.replace("\\\\","/");
    let mut xx0 = 0.0;
    for (pc, _rf, _cf, _an) in &bx {
        if let None = hpc.get(pc) {
            hpc.insert(pc, modlist[*pc].1.clone());
            println!("MAP_______  {} => {}", pc, modlist[*pc].1.clone());

            writeln!(&mut s, "file = '{}/{}'", cpt0, &modlist[*pc].0).expect("");
            writeln!(&mut s, "path = 'Object'").expect("");
            writeln!(&mut s, "name = '{}'", &modlist[*pc].1).expect("");
            writeln!(&mut s, r###"
bpy.ops.wm.append(
  filepath = os.path.join(file, path, name),
  directory= os.path.join(file, path),
  filename=name)
bpy.ops.transform.translate(value=({},-10,0))
            "###, xx0).expect("");
            xx0 += 0.5;
        }
    }

println!("mod num ..3");
    for (pc, rf, cf, an) in &bx {

		let dx = ax + rf * a + cf * c;
		let dy = ay + rf * b + cf * d;
		let co = format!("{},{},{}", dx, dy, az);
        writeln!(&mut s, "").unwrap();
        let obj0 = modlist[*pc].1.as_str();
		writeln!(&mut s, "bpy.ops.object.select_all(action='DESELECT')").unwrap();
        writeln!(&mut s, "bpy.data.objects['{}'].select_set(True)", obj0).unwrap();
        writeln!(&mut s, 
    "bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={{'linked':False, 'mode':'TRANSLATION'}})").unwrap();
        writeln!(&mut s, "bpy.ops.transform.rotate(value={}, orient_axis='Z')", an).unwrap();
		writeln!(&mut s, "for obj in bpy.context.selected_objects: obj.location = ({})", co).unwrap();
		writeln!(&mut s, "bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)").unwrap();
    }

println!("mod num ..4");
	let mut sf = format!("{}/{}.jpg", imtr, cnt);
	let mut tx = format!("{}/{}.txt", lbtr, cnt);
	if cnt%10==9 {
		sf = format!("{}/{}.jpg", imva, cnt);
		tx = format!("{}/{}.txt", lbva, cnt);
	} else if cnt%10==0 {
		sf = format!("{}/{}.jpg", imte, cnt);
		tx = format!("{}/{}.txt", lbte, cnt);
	}

	let chim = format!("{}/{}.jpg", toch, cnt);
	let chtx = format!("{}/{}.txt", toch, cnt);

	writeln!(&mut s, "bpy.context.scene.render.filepath = '{}/{}'",cpt(),sf).unwrap();
	writeln!(&mut s, "bpy.ops.render.render(write_still=True)").unwrap();
	let mut ss = String::new();
	for ii in 0..modlist.len() {
		let ob = &modlist[ii].1;
		println!(">>> {}", ob);
		write!(&mut ss, "\n    if o.name.find('{}')>=0 and len(o.name)>{} : ob={}", ob, ob.len(), ii).unwrap();
	}
	writeln!(&mut s, r###"
scn = bpy.context.scene
cam = bpy.data.objects['Camera']
f = open("{}", "w")
cn = 0
for o in bpy.data.objects:
    ob = -1;{}
    if ob<0 : continue

    lo = o.location
    co = bpy_extras.object_utils.world_to_camera_view(scn, cam, lo)
    co.y = 1.0 - co.y

    lf=-1; rg=-1; tp=-1; bt=-1;
    for v in o.data.vertices:
        co = lo + v.co
        co = bpy_extras.object_utils.world_to_camera_view(scn, cam, co)
        co.y = 1.0 - co.y
        if lf<0 or co.x<lf : lf = co.x
        if rg<0 or co.x>rg : rg = co.x
        if tp<0 or co.y<tp : tp = co.y
        if bt<0 or co.y>bt : bt = co.y
    ox = (lf+rg)/2
    oy = (tp+bt)/2
    wd = rg-lf
    hg = bt-tp
    txt = "%d %f %f %f %f\n" % (ob,ox,oy,wd,hg)
    f.write(txt)

f.close()
"###,tx,ss).unwrap();

println!("mod num ..5");
	let pyss = format!("{}/pys-{}.py", pys, cnt);
	println!("PYS: {}", pyss);
	let path = Path::new(&pyss);
    if let Ok(mut file) = File::create(&path) {
        file.write_all(s.as_bytes()).expect("");
    }

	let cmd = ["blender","-b",bld, "-P", &pyss];
	println!("  {:?}", cmd);

	let mut child = Command::new(cmd[0]).args(&cmd[1..]).spawn().unwrap();
	let _result = child.wait().unwrap();

	println!(" png: {}", &sf);

println!("mod num ..6");
	let mut tt = String::new();
	if let Ok(lines) = read_lines(&tx) {
		let mut img = image::open(&sf).unwrap();
		for line in lines {
			if let Ok(line) = line {
				let dds : Vec<f32> = line.split(" ")
					.map(|x| x.parse().expect("No float")).collect();
				let x = dds[1];
				let y = dds[2];
				let w = dds[3];
				let h = dds[4];
				let a = 640.0;
				
				let xc:i32 = (x * a) as i32;
				let yc:i32 = (y * a) as i32;
				
				let x1:i32 = ((x-w/2.0) * a) as i32;
				let y1:i32 = ((y-h/2.0) * a) as i32;
				let ww:u32 = (w * a) as u32;
				let hh:u32 = (h * a) as u32;

                if xc>3 && yc>3 && x1>0 && y1>0 {
				draw_hollow_rect_mut(&mut img
					, Rect::at(xc-3, yc-3).of_size(7, 7)
					, Rgba([255u8, 0u8, 0u8, 255u8]));
						
				draw_hollow_rect_mut(&mut img
					, Rect::at(x1, y1).of_size(ww, hh)
					, Rgba([255u8, 255u8, 255u8, 255u8]));
                }

				writeln!(&mut tt, "{}", &line).unwrap();
			}
		}
		img.save(chim).unwrap();
		
	}
println!("mod num ..7");
	let path = Path::new(&chtx);
    if let Ok(mut file) = File::create(&path) {
        file.write_all(tt.as_bytes()).expect("");
    }
}
*/

