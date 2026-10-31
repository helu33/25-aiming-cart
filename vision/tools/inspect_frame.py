"""PC-only detector preview and laser-sample template from one saved raw image."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from detector import FrameDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image")
    parser.add_argument("--output-prefix", required=True, help="New filename prefix; parent must exist")
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise ValueError("Cannot read image")
    height, width = image.shape[:2]
    detector = FrameDetector(width,height,config.DETECTION,
                             (config.TARGET_WIDTH_MM,config.TARGET_HEIGHT_MM),config.TAPE_WIDTH_MM)
    candidates = detector.candidates(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
    if not candidates:
        raise ValueError("No qualified frame. Check exposure, threshold, full border and tape width.")
    target = max(candidates,key=lambda c:c["area"])
    points = np.array(target["corners"],dtype=np.float32)
    overlay = image.copy()
    cv2.polylines(overlay,[np.rint(points).astype(np.int32)],True,(0,255,0),2)
    for label,point in zip(("TL","TR","BR","BL"),points):
        cv2.putText(overlay,label,tuple(np.rint(point).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
    # 2 px/mm diagnostic image of the outer edge (size from config).
    out_w, out_h = int(round(2*config.TARGET_WIDTH_MM)), int(round(2*config.TARGET_HEIGHT_MM))
    dest = np.array([[0,0],[out_w,0],[out_w,out_h],[0,out_h]],dtype=np.float32)
    warped = cv2.warpPerspective(image,cv2.getPerspectiveTransform(points,dest),(out_w+1,out_h+1))
    centre = tuple(int(round(2*v)) for v in config.TARGET_CENTER_MM)
    cv2.drawMarker(warped,centre,(0,255,0),cv2.MARKER_CROSS,18,1)
    outputs = [Path(args.output_prefix+s) for s in ("_overlay.png","_rectified.png","_sample.json")]
    if any(p.exists() for p in outputs):
        raise ValueError("Output exists; choose a new prefix")
    for path,img in zip(outputs,(overlay,warped)):
        if not cv2.imwrite(str(path),img):
            raise OSError("Cannot save " + str(path))
    sample = {"width":width,"height":height,"corners_px":[list(p) for p in target["corners"]],
              "spot_px":[None,None],"source_image":str(args.image)}
    with outputs[2].open("x",encoding="utf-8") as stream:
        json.dump(sample,stream,indent=2)
    print("Candidates:",len(candidates),"Selected largest. Verify green outline is the OUTER tape edge.")
    print("Fill spot_px with the measured spot center in the RAW image, then collect samples in one JSON.")
    for path in outputs: print(path)


if __name__ == "__main__":
    main()
