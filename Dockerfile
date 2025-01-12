# Use Python 3.11.9
FROM python:3.11.9-slim

# Set working directory
WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application
COPY . .

# Expose the port your app runs on
EXPOSE 8000

# Command to run the application using gunicorn
CMD ["gunicorn", "--bind", "0.0.0.0:8000", "app:app"] 