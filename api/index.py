from app import app

# Required for Vercel serverless
def handler(request, context):
    return app(request) 