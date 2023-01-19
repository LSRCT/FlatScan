import numpy as np
import os

def gen_imgfile(base_dir, format="rgb"):
    """
    Generate a text file listing all images in a directory
    """
    outfile = os.path.join(base_dir, f"{format}.txt")
    img_file_path = os.path.join(base_dir, format)
    assert os.path.exists(img_file_path), f"{img_file_path} does not exit!"
    print(f"Generating {outfile=}")

    files = os.listdir(img_file_path)
    files = sorted(files)
    timestamps = [float(x.split(".png")[0]) for x in files]
    timestamps = np.array(timestamps) / 1

    with open(outfile, "w") as f:
        for ts, img in zip(timestamps, files):
            f.write(f"{ts} {format}/{img}\n")

def gen_associations(base_dir, out_file="out.txt"):
    """
    Generate a text file associating rgb and depth images
    This assumes one to one correspondence between rgb and depth images
    DOES NOT TRY TO MATCH IMAGES BASED ON TIME STAMPS
    """
    outfile = os.path.join(base_dir, out_file)
    img_file_path = os.path.join(base_dir, "rgb")
    depth_file_path = os.path.join(base_dir, "depth")
    assert os.path.exists(base_dir), f"{base_dir} does not exit!"
    print(f"Generating {outfile=}")

    files_rgb = sorted(os.listdir(img_file_path))
    timestamps = [float(x.split(".png")[0]) for x in files_rgb]
    timestamps = np.array(timestamps) / 1
    files_rgb = [f"rgb/{x}" for x in files_rgb]
    files_depth = sorted(os.listdir(depth_file_path))
    files_depth = [f"depth/{x}" for x in files_depth]
    with open(outfile, "w") as f:
        for ts, img_rgb, img_d in zip(timestamps, files_rgb, files_depth):
            f.write(f"{ts} {img_rgb} {ts} {img_d}\n")
    

if __name__ == "__main__":
    base_path = "C:\\Users\\Alex\\Desktop\\Home\\discountDotCube\\KinectData\\ANeFull3"
    gen_imgfile(base_path, format="rgb")
    gen_imgfile(base_path, format="depth")
    gen_associations(base_path, out_file="anefull3.txt")