from django.shortcuts import render
from django.http import HttpResponse
from rest_framework import generics, status

from .serializers import UserSerializer
from .models import User

from rest_framework.views import APIView
from rest_framework.response import Response

# Create your views here.
class UserView(generics.ListAPIView):
    queryset = User.objects.all()
    serializer_class = UserSerializer