# Analyse du Repository SUPER - Notes Complètes

## Vue d'ensemble du système

SUPER est un système de navigation autonome haute vitesse pour drones développé par HKU MaRS Lab. Il utilise une architecture modulaire avec cartographie 3D, planification de trajectoire et gestion de missions.

## Architecture des modules principaux

### 1. ROG-Map (`rog_map/`) - Cartographie 3D
- **Fonction** : Génère une carte d'occupation 3D (occupancy grid map) à partir de données LiDAR
- **Topics d'entrée** :
  - `/cloud_registered` : Point cloud LiDAR (configurable via `cloud_topic` dans les YAML)
  - `/lidar_slam/odom` : Odométrie (configurable via `odom_topic`)
- **Cartographie probabiliste** : Chaque cellule a une probabilité d'être occupée (0.0 = libre, 1.0 = occupée)
- **Ray-casting** : Trace des rayons depuis le capteur vers chaque point détecté
  - Cellules traversées → marquées "libres" (probabilité diminuée)
  - Cellule finale → marquée "occupée" (probabilité augmentée)
- **Types de cartes générées** :
  - **Occupée** : Cellules contenant des obstacles détectés
  - **Inconnue** : Cellules jamais observées par les capteurs
  - **Gonflée (inflated)** : Carte occupée élargie d'une marge de sécurité pour la taille du drone

### 2. SUPER Planner (`super_planner/`) - Planification de Trajectoire 3D
- **Cœur du système** de planification
- **Algorithmes utilisés** :
  - **A*** : Recherche de chemin dans l'espace 3D
  - **CIRI** : Génération de corridors convexes sécurisés (Convex Iterative Region Inflation)
  - **Optimisation de trajectoire** : Trajectoires polynomiales avec contraintes dynamiques
- **Deux niveaux de trajectoires** :
  - **Trajectoire principale** (exp_traj) : optimisée pour la performance
  - **Trajectoire de secours** (backup_traj) : pour les situations d'urgence
- **Machine à états (FSM)** avec 6 états :
  - `INIT` : Initialisation du système
  - `WAIT_GOAL` : Attente d'un nouvel objectif
  - `YAWING` : Rotation vers l'orientation cible
  - `GENERATE_TRAJ` : Génération de trajectoire initiale
  - `FOLLOW_TRAJ` : Suivi de trajectoire avec replanification continue
  - `EMER_STOP` : Arrêt d'urgence

### 3. Mission Planner (`mission_planner/`) - Gestion des Missions
- **Fonction** : Gère les waypoints et objectifs de navigation
- **Interface** entre l'utilisateur et le planificateur
- **Modes supportés** :
  - "Click and go" via RViz
  - Missions prédéfinies via fichiers `.txt`
  - Contrôle via télécommande Mavros
- **Fonctionnement** :
  - Publie **un seul waypoint à la fois** sur `/planning/click_goal`
  - Attend que le drone s'approche (distance < `switch_distance`)
  - Passe automatiquement au waypoint suivant

### 4. MARS UAV Sim (`mars_uav_sim/`) - Simulateur
- **Simulateur de drone** avec rendu LiDAR réaliste
- **Génère des données de capteurs synthétiques**
- **Support** : différents types de LiDAR (360°, Livox AVIA, etc.)
- **Environnements** : utilise des cartes PCD pour les simulations

## Communication ROS et Topics

### Topics principaux
```
/cloud_registered          → Point cloud LiDAR vers ROG-Map
/lidar_slam/odom           → Odométrie vers ROG-Map et planificateur
/planning/click_goal       → Objectifs de navigation (point d'entrée principal)
/planning/pos_cmd          → Commandes de position vers contrôleur
/planning_cmd/poly_traj    → Trajectoires polynomiales (MPC)
```

### Flux de données
```
Perception (LiDAR/Caméra) → ROG-Map → Carte 3D → SUPER Planner
Mission Planner → /planning/click_goal → FSM → A* → CIRI → Trajectoire → Contrôleur
```

## Configuration et utilisation

### 1. Configuration des topics
- **Point cloud** : Modifier `cloud_topic` dans les fichiers YAML de `super_planner/config/`
- **Odométrie** : Modifier `odom_topic` dans les mêmes fichiers
- **Exemple** pour caméra stéréo : `cloud_topic: "/stereo/points2"`

### 2. Compatibilité ROS1/ROS2
- **Script de configuration** : `bash scripts/select_ros_version.sh ROS2`
- **À exécuter une seule fois** avant compilation
- **Modifie automatiquement** tous les `CMakeLists.txt` et `package.xml`

### 3. Lancement sans simulateur (drone réel)
```bash
# Terminal 1 - Système de perception (FAST-LIO, VIO, etc.)
roslaunch your_slam_package slam.launch

# Terminal 2 - Super Planner  
roslaunch super_planner fsm_node.launch

# Terminal 3 - Mission Planner (optionnel)
roslaunch mission_planner waypoint_mission.launch

# Terminal 4 - Visualisation
rviz -d super_planner/rviz/default.rviz
```

## Intégration avec votre projet d'essaim

### Options d'interaction avec le système

**Option A - Via Mission Planner :**
- Créer fichier `.txt` avec waypoints dans `mission_planner/data/`
- Format : `x y z switch_distance` par ligne

**Option B - Directement via topics ROS :**
```cpp
ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1);
geometry_msgs::PoseStamped goal;
goal.pose.position.x = target_x;
goal.pose.position.y = target_y; 
goal.pose.position.z = target_z;
goal.pose.orientation.w = 1.0;
goal.header.frame_id = "world";
goal_pub.publish(goal);
```

**Option C - Via RViz (tests) :**
- Outil "2D Nav Goal"

### Utilisation du ray-casting pour évitement d'obstacles
```cpp
// Utiliser l'API ROG-Map existante
rog_map::ROGMapROS::Ptr map_ptr = // votre instance
Vec3f robot_pos = // position du drone
Vec3f direction = // direction à tester
double max_distance = 5.0;

Vec3f end_point = robot_pos + direction * max_distance;
Vec3f collision_point;

bool is_free = map_ptr->isLineFree(robot_pos, end_point, collision_point, max_distance);
if (!is_free) {
    double obstacle_distance = (collision_point - robot_pos).norm();
    // Calculer vecteur d'évitement pour algorithme de boids
}
```

## Points importants pour l'implémentation

### 1. Modes du Mission Planner
Le Mission Planner supporte 3 modes de déclenchement (`start_trigger_type`) :
- **Mode 0 (RVIZ_CLICK)** : Déclenchement via clic dans RViz sur l'outil "2D Nav Goal"
- **Mode 1 (MAVROS_RC)** : Déclenchement via télécommande RC connectée à Mavros
  - Surveille le canal 10 de la RC (`msg->channels[9]`)
  - Se déclenche quand le canal passe de >1500 à <1500 (interrupteur basculé)
  - Permet contrôle à distance sans interface graphique
- **Mode 2 (TARGET_ODOM)** : Déclenchement automatique après délai (`start_program_delay`)

### 2. Rôles des topics de commande

**Topic `/planning/pos_cmd`** :
- **Publié par** : Super Planner FSM
- **Type** : `quadrotor_msgs::PositionCommand` (ROS1) ou `mars_quadrotor_msgs::msg::PositionCommand` (ROS2)
- **Contenu** : Commandes de position, vitesse, accélération, jerk, yaw pour le contrôleur PX4
- **Fréquence** : 100Hz (timer de 0.01s)
- **Rôle** : Interface directe avec le contrôleur de vol pour exécution immédiate

**Topic `/planning_cmd/poly_traj`** :
- **Publié par** : Super Planner FSM  
- **Type** : `quadrotor_msgs::PolynomialTrajectory` ou `mars_quadrotor_msgs::msg::PolynomialTrajectory`
- **Contenu** : Trajectoires polynomiales complètes avec coefficients
- **Rôle** : Pour contrôleurs MPC (Model Predictive Control) qui peuvent anticiper la trajectoire future

### 3. Répartition des rôles

**Mission Planner** :
- Gère la **séquence de waypoints** (un à la fois)
- **Publie les objectifs** sur `/planning/click_goal`
- **Ne fait PAS de planification** de chemin
- Optionnel si vous envoyez directement vos objectifs

**Super Planner** :
- **Reçoit les objectifs** via `/planning/click_goal`
- **Planifie le chemin** de la position actuelle vers l'objectif
- **Génère les trajectoires** optimisées
- **Publie les commandes** de vol

### 4. Repères de coordonnées et carte glissante
- **ROG-Map assume** que tous les points sont dans le **repère monde** (world frame)
- **Carte glissante activable** : `map_sliding: enable: true/false` dans les YAML
- **Si activée** : La carte "glisse" avec le drone (origine mobile)
- **Si désactivée** : Carte fixe avec origine définie (`fix_map_origin`)
- **Repère monde** : Référentiel fixe global, mais la carte peut être locale glissante

### 5. Conversion GPS vers repère local
```cpp
// Exemple de conversion GPS vers repère local
// 1. Définir une origine GPS de référence
double origin_lat = 48.8566;  // Paris par exemple
double origin_lon = 2.3522;
double origin_alt = 0.0;

// 2. Convertir GPS courant vers coordonnées métriques
double current_lat, current_lon, current_alt;
// ... obtenir GPS courant

// 3. Conversion approximative (pour petites distances)
double x = (current_lon - origin_lon) * 111320.0 * cos(origin_lat * M_PI/180.0);
double y = (current_lat - origin_lat) * 110540.0;
double z = current_alt - origin_alt;

// 4. Publier comme objectif
geometry_msgs::PoseStamped goal;
goal.pose.position.x = x;
goal.pose.position.y = y;
goal.pose.position.z = z;
```

### 6. Ignorer objets dynamiques (autres drones)
**Problème** : ROG-Map détecte tout comme obstacle statique

**Solutions possibles** :
1. **Filtrage par intensité** : Si vos drones ont des marqueurs réfléchissants spéciaux
2. **Filtrage temporel** : Modifier ROG-Map pour ignorer points qui bougent trop vite
3. **Zones d'exclusion** : Définir zones autour des positions connues des autres drones
4. **Communication inter-drones** : Partager positions et filtrer les points correspondants

**Modification suggérée dans ROG-Map** :
```cpp
// Dans la fonction de mise à jour du point cloud
for (auto& point : cloud) {
    // Vérifier si le point correspond à un autre drone connu
    if (isOtherDronePosition(point, known_drone_positions)) {
        continue; // Ignorer ce point
    }
    // Traitement normal du point
}
```

### 7. Compatibilité caméra stéréo
- **Compatible** avec n'importe quel `sensor_msgs/PointCloud2`
- **Configurer** le bon topic dans `cloud_topic`
- **S'assurer** que le point cloud est publié dans le repère monde

### 8. Topic `/cloud_registered`
- **Nom standard** utilisé par FAST-LIO/FAST-LIO2
- **Pas universel** en ROS2, mais convention dans l'écosystème HKU MaRS Lab
- **Configurable** selon votre système de perception

## Intégration avec algorithme de boids

Le système SUPER peut servir de base pour votre essaim :

1. **Navigation individuelle** : Chaque drone utilise SUPER pour éviter obstacles statiques
2. **Cartographie partagée** : ROG-Map extensible pour fusion de cartes multi-drones
3. **Planification coopérative** : Planificateur modifiable pour intégrer règles de boids
4. **Évitement de collision** : Corridors CIRI adaptables pour éviter autres drones
5. **Ray-casting** : API existante utilisable pour détection d'obstacles proches et calcul de vecteurs d'évitement

L'architecture modulaire facilite l'intégration de votre algorithme de boids au niveau du planificateur.
