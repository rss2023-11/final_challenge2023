import torch

model = torch.load('yolov5n.pt')
torch.save('yolov5n_statedict.pt', model.state_dict())