from django.urls import path

from . import views

app_name = 'control'
urlpatterns = [
    path('', views.index, name='index'),
    path('command/', views.command, name='command'),
    path('stoprecord/', views.stoprecord, name='stoprecord'),
    path('random/', views.random, name='random'),
    path('startrecord/', views.startrecord, name='startrecord'),
]
