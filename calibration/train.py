import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import torch
import torch.nn as nn
import torch.optim as optim
import joblib

torch.manual_seed(42)
np.random.seed(42)

file_num = 0 # Change your file number

# Load the data from CSV
filename = f'./calibration/sensor_data_log_{file_num}.csv'  # Replace with your file path
data = pd.read_csv(filename)

# Extract input (H_matrix) and output (F_matrix)
H_matrix = data.iloc[30:, 7:10].values  # Columns for hall sensor data (data1~data4)
F_matrix = data.iloc[30:, 1:4].values   # Columns for known force vectors (RFT_1, RFT_2, RFT_3)

# Split the data into training and testing sets
H_train, H_test, F_train, F_test = train_test_split(H_matrix, F_matrix, test_size=0.2, random_state=42)

# Scale the data for better training stability
scaler_x = StandardScaler()
H_train = scaler_x.fit_transform(H_train)
H_test = scaler_x.transform(H_test)
scaler_y = StandardScaler()
F_train = scaler_y.fit_transform(F_train)
F_test = scaler_y.transform(F_test)

# Save the scaler for later use
joblib.dump(scaler_x, f'./calibration/scalerX_{file_num}.pkl')
joblib.dump(scaler_y, f'./calibration/scalerY_{file_num}.pkl')

# Convert data to PyTorch tensors
H_train_tensor = torch.tensor(H_train, dtype=torch.float32)
F_train_tensor = torch.tensor(F_train, dtype=torch.float32)
H_test_tensor = torch.tensor(H_test, dtype=torch.float32)
F_test_tensor = torch.tensor(F_test, dtype=torch.float32)

# Define the MLP model in PyTorch
class MLPModel(nn.Module):
    def __init__(self):
        super(MLPModel, self).__init__()
        self.fc1 = nn.Linear(3, 32)  # Input layer with 3 neurons
        self.fc2 = nn.Linear(32, 16)  # Hidden layer with 16 neurons
        self.fc3 = nn.Linear(16, 3)  # Output layer with 3 neurons

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# Initialize the model, loss function, and optimizer
model = MLPModel()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Training the model
epochs = 200
batch_size = 16
train_size = H_train_tensor.size(0)

for epoch in range(epochs):
    model.train()
    permutation = torch.randperm(train_size)
    
    for i in range(0, train_size, batch_size):
        indices = permutation[i:i + batch_size]
        batch_H = H_train_tensor[indices]
        batch_F = F_train_tensor[indices]
        
        optimizer.zero_grad()
        outputs = model(batch_H)
        loss = criterion(outputs, batch_F)
        loss.backward()
        optimizer.step()
    
    if (epoch + 1) % 10 == 0:
        print(f"Epoch {epoch+1}/{epochs}, Loss: {loss.item():.4f}")

# Evaluate the model on the test set
model.eval()
with torch.no_grad():
    predictions = model(H_test_tensor)
    test_loss = criterion(predictions, F_test_tensor)
print(f"Test MSE: {test_loss.item():.4f}")

# Save the trained model
torch.save(model.state_dict(), f'./calibration/mlp_model_{file_num}.pth')  # Replace with your file path
