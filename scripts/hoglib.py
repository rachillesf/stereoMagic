#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import cv2
import glob
import os


def non_max_suppression(boxes, overlapThresh):
	# testa se há janelas detectadas, se não tiver retorna um erro e para o programa
    try:
        len(boxes) != 0
    except:
        print("ERROR: empty array boxes")

	# se as coordenadas das janelas forem inteiros transforma em float
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")

	# inicia o vetor de indexes
	pick = []

	# separa as coordenadasem vetores, ponto superior direito (x1,y1) e inferior esquerdo (x2,y2)
	x1 = boxes[:, 0]
	y1 = boxes[:, 1]
	x2 = boxes[:, 2]
	y2 = boxes[:, 3]

	# calcula a área das janelas
	area = (x2 - x1 + 1) * (y2 - y1 + 1)
	# retorna os indexes ordenados, suas ordens corretas
	idxs = np.argsort(y2)

	# mantem a pesquisa por janelas enquanto houver indexes
	while len(idxs) > 0:
		# pega a posição do ultimo index e adiciona o index a lista de indexes capturados
		last=len(idxs) - 1
		i = idxs[last]
		pick.append(i)

		# encontra a maior coordenada para criar a janela, e a menor coordenada
		x1_max = np.maximum(x1[i], x1[idxs[:last]])
		y1_max = np.maximum(y1[i], y1[idxs[:last]])
		x2_min = np.minimum(x2[i], x2[idxs[:last]])
		y2_min = np.minimum(y2[i], y2[idxs[:last]])

		# calcula altura e comprimento da janela
		w = np.maximum(0, x2_min - x1_max + 1)
		h = np.maximum(0, y2_min - y1_max + 1)

		# calcula a razão de sobreposição
		overlap = (w * h) / area[idxs[:last]]

		# apaga os indexes com razão de sobreposição maiores que o limiar fornecido
		idxs = np.delete(idxs, np.concatenate(([last],np.where(overlap > overlapThresh)[0])))
        # retorna apenas as janelas finais com valor int
        return boxes[pick].astype("int")




def detectPeople(image):
    orig = image.copy()

    # inicializa o descritor HOG e o detector de pessoas com SVM pre-treinado
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


    (h, w) = image.shape[:2]
    r = 640 / float(w)
    dim = (640, int(h * r))
    #resize the image
    image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)



    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image,winStride=(4,4),padding=(16,16),scale=1.05)

    # draw the original bounding boxes
    for (x, y, w, h) in rects:
	       cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, overlapThresh=0.48)#0.48

    # draw the final bounding boxes
    # show some information on the number of bounding boxes
    try:
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

    except:
        print("nao foi")
        pass


	# show the output images
    #cv2.imshow("Before NMS", orig)
    #cv2.imshow("After NMS", image)
    #cv2.imwrite('result-bike.png',image)
    #cv2.waitKey(3)

    return pick, image


#image = cv2.imread('imagem.png')
#detectPeople(image)
