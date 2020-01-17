from django.http import HttpResponse, Http404, HttpResponseRedirect
from django.shortcuts import get_object_or_404, render
from django.urls import reverse

import os

def index(request):
    path = "/home/rosie/Videos"  # insert the path to your directory
    img_list = os.listdir(path)
    return render(request, 'review/index.html', {'images': img_list})