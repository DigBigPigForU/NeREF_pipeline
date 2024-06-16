import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, ReLU
from tensorflow.keras.optimizers import Adam
from sklearn.model_selection import train_test_split

# Load your data (assuming x and y are already in your workspace as NumPy arrays)
# For example:
# x = np.load('x.npy')
# y = np.load('y.npy')

# Check the shape of variables
print(x.shape)
print(y.shape)

# Split data into training and validation sets
x_train, x_val, y_train, y_val = train_test_split(x, y, test_size=0.2, random_state=42)

# Define the model architecture
NeREF = Sequential([
    Dense(128, input_shape=(x.shape[1],)),  # Input layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Second hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Third hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Fourth hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Fifth hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Sixth hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Seventh hidden layer
    ReLU(),                                 # Activation function
    Dense(128),                             # Eighth hidden layer
    ReLU(),                                 # Activation function
    Dense(y.shape[1])                       # Output layer
])

# Compile the model
NeREF.compile(optimizer=Adam(learning_rate=0.001),
              loss='mean_squared_error')

# Train the model
history = NeREF.fit(x_train, y_train,
                    validation_data=(x_val, y_val),
                    epochs=50,
                    batch_size=64,
                    verbose=1)

# Save the model
NeREF.save('NeREF.h5')
