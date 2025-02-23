from torch.utils.data import DataLoader


def create_dataloaders(train_dataset, valid_dataset, test_dataset, batch_size, **kwargs):
    kwargs["batch_size"] = batch_size

    valid_dataloader = DataLoader(valid_dataset, **kwargs)
    test_dataloader = DataLoader(test_dataset, **kwargs)

    kwargs.setdefault("shuffle", True)
    train_dataloader = DataLoader(train_dataset, **kwargs)

    return train_dataloader, valid_dataloader, test_dataloader
