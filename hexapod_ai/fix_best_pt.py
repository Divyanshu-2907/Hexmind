import torch
import yaml

def main():
    yaml_path = r"c:\Users\dk290\OneDrive\Desktop\Hexamind\hexapod_ai\dataset\dataset_v2\data.yaml"
    model_path = r"c:\Users\dk290\OneDrive\Desktop\Hexamind\hexapod_ai\models\best.pt"

    # 1. Read the real names from your newly found dataset_v2 yaml
    print(f"Reading names from: {yaml_path}")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    names_list = data.get('names', [])
    names_dict = {i: name for i, name in enumerate(names_list)}
    
    print(f"Found {len(names_dict)} classes! ({names_dict[0]}, {names_dict[1]}... up to {names_dict[len(names_dict)-1]})")

    # 2. Open the best.pt file
    print(f"Loading model metadata from: {model_path}")
    import warnings
    warnings.simplefilter('ignore')
    ckpt = torch.load(model_path, map_location="cpu", weights_only=False)

    # 3. Overwrite the amnesic "class_N" names with the real names
    if "model" in ckpt and hasattr(ckpt["model"], "names"):
        ckpt["model"].names = names_dict
        
        # 4. Save the repaired model
        torch.save(ckpt, model_path)
        print("\n\u2705 SUCCESS! best.pt is now permanently fixed! It will now always output real names.")
    else:
        print("\n\u274c FAILED! Could not find 'names' attribute in the model.")

if __name__ == "__main__":
    main()
