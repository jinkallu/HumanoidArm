import torch
import torch.nn as nn


class PCN(nn.Module):
    """
    "PCN: Point Cloud Completion Network"
    (https://arxiv.org/pdf/1808.00671.pdf)

    Attributes:
        num_dense:  16384
        latent_dim: 1024
        grid_size:  4
        num_coarse: 1024
    """

    def __init__(self, num_dense=16384, latent_dim=1024, grid_size=4):
        super().__init__()

        self.num_dense = num_dense
        self.latent_dim = latent_dim
        self.grid_size = grid_size

        assert self.num_dense % self.grid_size ** 2 == 0

        self.num_coarse = self.num_dense // (self.grid_size ** 2)

        self.first_conv = nn.Sequential(
            nn.Conv1d(3, 128, 1),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.Conv1d(128, 256, 1)
        )

        self.second_conv = nn.Sequential(
            nn.Conv1d(512, 512, 1),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Conv1d(512, self.latent_dim, 1)
        )

        self.mlp = nn.Sequential(
            nn.Linear(self.latent_dim, 1024),
            nn.ReLU(inplace=True),
            nn.Linear(1024, 1024),
            nn.ReLU(inplace=True),
            nn.Linear(1024, 3 * self.num_coarse)
        )

        self.final_conv = nn.Sequential(
            nn.Conv1d(1024 + 3 + 2, 512, 1),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Conv1d(512, 512, 1),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Conv1d(512, 3, 1)
        )
        a = torch.linspace(-0.05, 0.05, steps=self.grid_size, dtype=torch.float).view(1, self.grid_size).expand(self.grid_size, self.grid_size).reshape(1, -1)
        b = torch.linspace(-0.05, 0.05, steps=self.grid_size, dtype=torch.float).view(self.grid_size, 1).expand(self.grid_size, self.grid_size).reshape(1, -1)
        
        self.folding_seed = torch.cat([a, b], dim=0).view(1, 2, self.grid_size ** 2)  # (1, 2, S)

    def forward(self, xyz):
        B, N, _ = xyz.shape
        
        # encoder
        feature = self.first_conv(xyz.transpose(2, 1).to(xyz.device))                                       # (B,  256, N)
        feature_global = torch.max(feature, dim=2, keepdim=True)[0].to(xyz.device)                         # (B,  256, 1)
        feature = torch.cat([feature_global.expand(-1, -1, N), feature], dim=1).to(xyz.device)              # (B,  512, N)
        feature = self.second_conv(feature).to(xyz.device)                                                  # (B, 1024, N)
        feature_global = torch.max(feature,dim=2,keepdim=False)[0].to(xyz.device)                           # (B, 1024)
        
        # decoder
        coarse = self.mlp(feature_global).reshape(-1, self.num_coarse, 3).to(xyz.device)                    # (B, num_coarse, 3), coarse point cloud
        point_feat = coarse.unsqueeze(2).expand(-1, -1, self.grid_size ** 2, -1).to(xyz.device)             # (B, num_coarse, S, 3)
        point_feat = point_feat.reshape(-1, self.num_dense, 3).transpose(2, 1).to(xyz.device)               # (B, 3, num_fine)

        seed = self.folding_seed.unsqueeze(2).expand(B, -1, self.num_coarse, -1).to(xyz.device)             # (B, 2, num_coarse, S)
        seed = seed.reshape(B, -1, self.num_dense).to(xyz.device)                                           # (B, 2, num_fine)

        feature_global = feature_global.unsqueeze(2).expand(-1, -1, self.num_dense).to(xyz.device)          # (B, 1024, num_fine)

        # Check devices before concatenation
        # print(f"feature_global device: {feature_global.device}, shape: {feature_global.shape}")
        # print(f"seed device: {seed.device}, shape: {seed.shape}")
        # print(f"point_feat device: {point_feat.device}, shape: {point_feat.shape}")

        feat = torch.cat([feature_global, seed, point_feat], dim=1).to(xyz.device)                          # (B, 1024+2+3, num_fine)
    
        fine = self.final_conv(feat).to(xyz.device) + point_feat.to(xyz.device)                                            # (B, 3, num_fine), fine point cloud

        return coarse.contiguous().to(xyz.device), fine.transpose(1, 2).contiguous().to(xyz.device)
