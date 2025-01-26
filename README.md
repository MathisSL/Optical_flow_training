# Algorithme de Lucas-Kanade

L'algorithme de **Lucas-Kanade** est une méthode classique utilisée pour estimer le flux optique, c'est-à-dire le mouvement des objets entre deux images successives. Il est couramment utilisé dans le suivi d'objets et la détection de mouvement dans des vidéos. Cet algorithme suppose que le mouvement des objets est faible entre deux images successives et que la variation d'intensité des pixels suit une relation linéaire.

![youtube-video-gif(12)](https://github.com/user-attachments/assets/a17fa757-14c6-4b0a-97d9-df8d9192c21e)


## Principe de fonctionnement

L'algorithme de Lucas-Kanade repose sur deux hypothèses principales :
1. **Hypothèse de constance de l'intensité** : Les valeurs d'intensité d'un pixel entre deux images consécutives ne changent pas de manière significative (le mouvement est petit).
2. **Hypothèse de petit mouvement** : Le mouvement des objets entre les deux images est suffisamment petit pour que le déplacement d'un pixel puisse être approximé par une transformation linéaire.

### Étapes de l'algorithme de Lucas-Kanade

1. **Calcul des gradients d'intensité** :
   L'algorithme commence par calculer les gradients d'intensité de l'image dans les directions $\( x \)$ et $\( y \)$ (les dérivées partielles de l'image par rapport aux coordonnées $\( x \)$ et $\( y \))$. Ces gradients représentent le taux de variation de l'intensité à chaque pixel.

   - Le gradient dans la direction $\( x \)$ $(\( I_x \))$ et dans la direction $\( y \)$ $(\( I_y \))$ sont calculés en utilisant des filtres de Sobel, ou d'autres méthodes de dérivation approximée.

2. **Calcul du terme de flux optique** :
   Pour chaque pixel, un petit voisinage de pixels est utilisé pour estimer le flux optique. L'idée est d'estimer les vitesses horizontale (\( u \)) et verticale (\( v \)) du mouvement à partir des gradients d'intensité.

   L'équation de base du flux optique est donnée par :


   $I_x \cdot u + I_y \cdot v + I_t = 0$

   où :
   - $\( I_x \)$ et $\( I_y \)$ sont les gradients spatiaux dans les directions $\( x \)$ et $\( y \)$,
   - $\( I_t \)$ est le gradient temporel, qui représente la différence d'intensité entre les deux images.


4. **Résolution de l'équation** :
   L'algorithme de Lucas-Kanade résout cette équation pour un voisinage local de chaque pixel. Cela donne les composantes du mouvement $\( u \)$ (déplacement horizontal) et $\( v \)$ (déplacement vertical) à chaque pixel.

   Une approche courante consiste à résoudre cette équation par une méthode de moindres carrés pour chaque pixel dans un petit voisinage (par exemple, une fenêtre de \( 3 \times 3 \)).

5. **Estimation du flux optique** :
   Le mouvement est estimé pour chaque pixel en fonction des composantes $\( u \)$ et $\( v \)$ calculées, et ce processus est répété pour chaque image de la séquence.

## Application et Utilisation

L'algorithme de Lucas-Kanade est couramment utilisé dans des applications de vision par ordinateur, telles que :
- Le suivi de mouvements dans des vidéos (par exemple, suivi d'objets).
- L'estimation du flux optique pour la reconstruction 3D.
- La stabilisation de vidéos.
- L'analyse du mouvement dans des séquences d'images.

## Avantages et Limites

### Avantages :
- **Simple et rapide** : L'algorithme de Lucas-Kanade est relativement simple à implémenter et efficace dans les cas de mouvement modéré.
- **Robuste** : Il est robuste aux petites variations d'intensité entre les images.

### Limites :
- **Mouvement faible** : L'algorithme suppose que le mouvement est faible entre les images successives. Il n'est donc pas adapté aux grandes variations de mouvement.
- **Problèmes avec les zones sans texture** : Il peut échouer dans des régions de l'image avec peu de détails ou de texture (par exemple, des zones uniformes ou plates).

## Conclusion

L'algorithme de Lucas-Kanade est une méthode classique et efficace pour l'estimation du flux optique dans des applications de vision par ordinateur. Bien qu'il soit limité par l'hypothèse de petits mouvements, il reste une solution populaire pour de nombreux problèmes pratiques de suivi de mouvement et d'analyse d'images.

# Filtre de Kalman

Le **filtre de Kalman** est un algorithme récursif utilisé pour estimer l'état d'un système dynamique à partir d'observations bruitées. Il est largement utilisé dans des domaines tels que la robotique, la navigation, la finance, et les systèmes de suivi. L'idée principale est de prédire l'état futur d'un système tout en réduisant les erreurs dues au bruit et aux incertitudes des mesures.

![youtube-video-gif(11)](https://github.com/user-attachments/assets/a59b6ea0-00c3-43ce-bab7-abc17da2cd89)


## Principe de fonctionnement

Le filtre de Kalman repose sur deux grandes étapes : la **prédiction** et la **correction**. Il combine les informations provenant d'un modèle du système (prédiction) et des mesures bruitées (correction) pour fournir une estimation optimale de l'état du système à chaque instant.

### 1. **Modèle du système**

Le filtre de Kalman modélise l'état du système et son évolution à travers les équations suivantes :

#### Équation d'état :


$x_k = A \cdot x_{k-1} + B \cdot u_k + w_k$

Où :
- $\( x_k \)$ est l'état du système à l'instant $\( k \)$,
- $\( A \)$ est la matrice de transition d'état qui décrit l'évolution de l'état entre deux instants,
- $\( B \)$ est la matrice d'entrée qui modélise l'impact des contrôles ou commandes $\( u_k \)$,
- $\( w_k \)$ est le bruit du processus, généralement supposé être gaussien et de moyenne nulle.

#### Équation de mesure :

$z_k = H \cdot x_k + v_k$

Où :
- $\( z_k \)$ est la mesure obtenue à l'instant $\( k \)$,
- $\( H \)$ est la matrice de mesure qui décrit comment l'état est observé,
- $\( v_k \)$ est le bruit de mesure, également supposé être gaussien avec une moyenne nulle.

### 2. **Prédiction**

À chaque itération, le filtre de Kalman commence par prédire l'état du système en utilisant le modèle d'évolution (équation d'état). Il calcule également la covariance de l'estimation de l'état, qui mesure l'incertitude associée à l'estimation.

$\hat{x}_k^- = A \cdot \hat{x}_{k-1} + B \cdot u_k$

$P_k^- = A \cdot P_{k-1} \cdot A^T + Q$

Où :
- $\( \hat{x}_k^- \)$ est l'état prédit,
- $\( P_k^- \)$ est la covariance de l'estimation prédite,
- $\( Q \)$ est la matrice de covariance du bruit de processus.

### 3. **Correction (Mise à jour)**

Une fois la mesure $\( z_k \$ disponible, le filtre de Kalman met à jour l'estimation de l'état en combinant la prédiction avec la mesure. La mise à jour se fait en calculant un gain de Kalman optimal, qui détermine la manière de pondérer la prédiction et la mesure.

$K_k = P_k^- \cdot H^T \cdot (H \cdot P_k^- \cdot H^T + R)^{-1}$

$\hat{x}_k = \hat{x}_k^- + K_k \cdot (z_k - H \cdot \hat{x}_k^-)$

$P_k = (I - K_k \cdot H) \cdot P_k^-$

Où :
- $\( K_k \)$ est le gain de Kalman,
- $\( R \)$ est la matrice de covariance du bruit de mesure,
- $\( \hat{x}_k \)$ est l'estimation mise à jour de l'état,
- $\( P_k \)$ est la covariance mise à jour de l'estimation.

### 4. **Récursivité**

Le filtre de Kalman est récursif, ce qui signifie qu'il met constamment à jour son estimation de l'état à chaque nouvelle mesure. Cette propriété est particulièrement utile pour les systèmes en temps réel, car l'estimation peut être ajustée en continu sans avoir besoin de recalculer les états passés.

## Application et Utilisation

Le filtre de Kalman est utilisé dans de nombreuses applications, notamment :
- **Suivi de mouvement** : Pour estimer la position et la vitesse d'un objet (par exemple, suivi de véhicule, robot mobile, etc.).
- **Navigation** : Pour l'estimation de la position et de la vitesse d'un véhicule (avion, sous-marin, etc.) en utilisant des capteurs comme le GPS, les accéléromètres et les gyroscopes.
- **Filtrage de données bruitées** : Pour réduire l'impact du bruit dans des signaux ou mesures, par exemple en traitement d'image ou en analyse financière.

## Avantages et Limites

### Avantages :
- **Optimisation** : Le filtre de Kalman fournit l'estimation optimale de l'état sous certaines hypothèses (modèle linéaire, bruit gaussien).
- **Récursivité** : Le calcul est effectué de manière récursive, ce qui permet une estimation continue à partir de nouvelles mesures.
- **Efficacité** : Il est relativement peu coûteux en termes de calculs, ce qui le rend adapté aux systèmes en temps réel.

### Limites :
- **Modèle linéaire** : Le filtre de Kalman classique suppose un modèle linéaire du système et des mesures. Cependant, il existe des extensions pour gérer les systèmes non linéaires (par exemple, le filtre de Kalman étendu).
- **Hypothèses sur le bruit** : Le filtre de Kalman suppose que les bruits de processus et de mesure sont gaussiens, ce qui peut ne pas être vrai dans certains cas.

## Conclusion

Le filtre de Kalman est un outil puissant pour l'estimation d'états dans des systèmes dynamiques soumis à des incertitudes et à des bruits. Il est utilisé dans de nombreuses applications en raison de son efficacité, de sa capacité à fournir des estimations optimales et de son caractère récursif, ce qui en fait un choix privilégié pour des systèmes en temps réel.

