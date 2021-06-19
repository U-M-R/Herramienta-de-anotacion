# Herramienta-de-anotación
Herramienta para anotar imágenes a partir de un archivo .bag (ROS)

# Iniciar la herramienta:

python3 anotacion.py <ruta_archivo_bag> <tópico>
  
Ejemplo: python3 anotacion.py /home/nombre/Downloads/coche.bag /cv_camera/image_raw

# Uso:

  Esta herramienta utiliza determinadas teclas para anotar distintos objetos.
  
  Espacio --> Detiene el bag en el frame actual.
  
  a --> Volver al frame anterior.
  
  d --> Pasar al frame siguiente.
  
  t --> Se obtiene la posición actual del puntero (Será el inicio de la anotación del objeto)
  
  c --> Si la tecla t ha sido pulsada, se obtendrá la posición actual del puntero para terminar la anotación del objeto, al haber pulsado la tecla c, habremos anotado un coche.
  
  p --> Ídem de la tecla c, pero en este caso será una persona.
  
  s --> Ídem de la tecla c, pero en este caso será una señal de tráfico.


### Aún en desarrollo.
