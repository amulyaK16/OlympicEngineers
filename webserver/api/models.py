from django.db import models
from django.db.models.fields import DateTimeField

# Create your models here.
class User(models.Model):
    username = models.CharField(max_length=20, unique=True)
    password = models.CharField(max_length=20)
    email = models.EmailField(max_length=254)
    created_at = models.DateTimeField(auto_now_add=True)
