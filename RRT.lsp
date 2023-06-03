; Variables setup
(setq l1 150.0  ; length of link 1
      l2 100.0) ; length of link 2


; Define obstacles
(setq obstacles (list (list (list 205.0 115.0) 10)

		      (list (list -50.0 110.0) 5)))
; Opora point
(setq base-point (list 0.0 0.0))


; Define the configuration space bounds
(setq qmin (list (- (/ pi 2)) (- (/ pi 2))))
(setq qmax (list (/ pi 2) (/ pi 2)))


; Define the initial configuration
(setq q0 (list 0.0 0.0))


; Define the goal configuration
(setq goal (list 1.0 1.0))




; FUNCTION DEFINITIONS


; Define the robot arm kinematics
(defun forward-kinematics (theta1 theta2 / x y)
  (setq x 0.0  ; x-coordinate of end effector
        y 0.0) ; y-coordinate of end effector
  (setq x (+ (* l1 (cos theta1)) (* l2 (cos (+ theta1 theta2)))))
  (setq y (+ (* l1 (sin theta1)) (* l2 (sin (+ theta1 theta2)))))
  (list x y)
)


; Calculate the distance between two points
(defun euclidean-distance (q1 q2 / p1 p2 dx dy)
  (setq p1 (forward-kinematics (nth 0 q1) (nth 1 q1))
	p2 (forward-kinematics (nth 0 q2) (nth 1 q2)))
  (setq dx (- (nth 0 p1) (nth 0 p2)))
  (setq dy (- (nth 1 p1) (nth 1 p2)))
  (sqrt (+ (* dx dx) (* dy dy)))
)


; Check if the random generated point is close enough to the other points in the trees
(defun distance-check (qnear qrand delta / distance-new-point theta qnew)
  (setq distance-new-point (euclidean-distance qnear qrand))
  (if (< distance-new-point delta) qrand
    (progn
      (setq theta (atan (- (nth 1 qrand) (nth 1 qnear))
                        (- (nth 0 qrand) (nth 0 qnear))))
      (setq qnew (list (+ (nth 0 qnear) (* delta (cos theta)))
                       (+ (nth 1 qnear) (* delta (sin theta)))))
  qnew))
)


; Find the nearest neighbor of a chosen point
(defun nearest-neighbor (q tree / nearest min-distance qn d)
  (setq nearest nil)
  (setq min-distance nil)
  (foreach node tree
    (setq qn (car node))
    (setq d (euclidean-distance q qn))
    (if (or (null min-distance) (< d min-distance)) 
        (progn (setq nearest qn)
               (setq min-distance d))))
  nearest
)


; Check for collision with a chosen robot arm link
(defun check-collisions (p1 p2 B n / a0 a L R ang collision)
  (setq L (distance p1 p2)
	a0 0.0
	a (/ L (* 2.0 n))
	ang (angle p1 p2)
	R  (sqrt (+ (* (/ B 2.0) (/ B 2.0)) (* a a)))
	collision nil
  )
  (repeat (+ n 1)
    (foreach obst obstacles
      (if (< (distance p1 (nth 0 obst)) (+ R (nth 1 obst))) (setq collision t))
    )
    (setq p1 (polar p1 ang (+ a0 (* a 2.0))))
  )
  collision
)


; Check if a collision occured
(defun notCollision (q / p1 p2)
  (setq p1 (polar base-point (nth 0 q) l1)
	p2 (polar p1 (+ (nth 0 q) (nth 1 q)) l2))
    
    (and (not (check-collisions base-point p1 40.0 7))
	 (not (check-collisions p1 p2 40.0 7)))
)


; Check if the configuration is valid
(defun valid-configuration (q hand-restriction)
  (and (<= (nth 0 qmin) (nth 0 q) (nth 0 qmax))
       (<= (nth 1 qmin) (nth 1 q) (nth 1 qmax))
       (<= (nth 0 hand-restriction) (nth 1 q) (nth 1 hand-restriction))
       (notCollision q))
)


; RNG function
(defun rnd (/ modulus multiplier increment random)
  (if (not seed)
    (setq seed (getvar "DATE"))
  )
  (setq modulus    65536
        multiplier 25173
        increment  13849
        seed  (rem (+ (* multiplier seed) increment) modulus)
        random     (/ seed modulus)
  )
)


; Generate random angle between two chosen angles
(defun random-angle (angle1 angle2)
  (+ angle1 (* (rnd) (- angle2 angle1)))
)


; Define the robot arm angle restrictions
(defun hand-settings (hand / hand-restriction)
  (setq hand-restriction (list (- (/ pi 2)) (/ pi 2)))
  (if (= hand "right")
    (setq hand-restriction (list 0 (/ pi 2)))
  )
  (if (= hand "left")
    (setq hand-restriction (list (- (/ pi 2)) 0))
  )
  hand-restriction)


;(node q parent)
; Define the rrt algorithm
(defun rrt (qgoal max-iterations delta hand tree / continue qrand qnear qnew i hand-restriction)
  ; Declare the variables used in the function
  (setq hand-restriction (hand-settings hand))
  (setq i 0)
  (while (and (< i max-iterations) (not goal-found))
    (setq continue nil)
    (setq qrand (list (random-angle (nth 0 qmin) (nth 0 qmax))
                      (random-angle (nth 1 qmin) (nth 1 qmax))))
    (if (not (valid-configuration qrand hand-restriction))
        (progn
          (setq i (+ i 1))
          (setq continue t)))
    (if (and (not continue))
      (progn
        (setq qnear (nearest-neighbor qrand tree))
        (setq qnew (distance-check qnear qrand delta))
        (if (not (valid-configuration qnew hand-restriction))
            (progn
              (setq i (+ i 1))
              (setq continue t)))
        (if (and (< (euclidean-distance qnew qgoal) delta) (not continue))
            (progn (setq goal-found t)
                   (setq tree (append tree (list (list qnew qnear))))
                   (setq i (+ i 1))
                   (setq continue t)))
        (if (and (not continue))
	  (progn
            (setq tree (append tree (list (list qnew qnear))))
            (setq i (+ i 1))
	  )
        )
      )
    )
  )
  (if goal-found (setq tree (append tree (list (list qgoal qnew)))))
  (list tree goal-found)
)


; This function wraps the rrt algorithm in order to preserve the progress of the tree building
(defun rrt-wrapper (q0 qgoal max-iterations delta hand / tree goal-found results)
  (setq tree (list (list q0 nil)))
  (setq goal-found nil)
  (setq results (rrt qgoal max-iterations delta hand tree))
  (setq tree (nth 0 results))
  (setq goal-found (nth 1 results))
  (if (not goal-found)
    (progn
      (if (= hand "right")
	(progn
	  (setq hand "left")
	  (setq results (rrt qgoal max-iterations delta hand tree))
	  ))
      (if (= hand "left")
	(progn
	  (setq hand "right")
	  (setq results (rrt qgoal max-iterations delta hand tree))
	  ))
      (setq goal-found (nth 1 results))
      (if (not goal-found)
	(setq tree nil)
	(setq tree (nth 0 results))
	)
      )
    )
  tree
)


; Create a list of points containing the path from the start to the goal
(defun extract-path (last-node tree / config parent)
  (if (null last-node)
    '()
    (progn
      (setq config (car last-node)
            parent (cadr last-node))
      (cons config (extract-path (assoc parent tree) tree))
    )
  )
)





; MAIN

; Run the RRT algorithm
(setq build-tree (rrt-wrapper q0 goal 4000 50 "right"))
(setq path (reverse (extract-path (last build-tree) build-tree)))


; Draw path
(defun draw-robot-path (path / p1 p2 q obst)
  (opora base-point 8)
  (make-c (forward-kinematics (nth 0 q0) (nth 1 q0)) 5.0 2 "path")
  (make-c (forward-kinematics (nth 0 goal) (nth 1 goal)) 5.0 3 "path")
  (print-polyline-path path)
  (foreach obst obstacles
    (make-c (nth 0 obst) (nth 1 obst) 1 "obst")
  )
  (foreach q path
    (animate "Link")
    (animate "circle")
    (setq p1 (polar base-point (nth 0 q) l1)
	  p2 (polar p1 (+ (nth 0 q) (nth 1 q)) l2))
    (link base-point p1)
    (link p1 p2)
    
    (fill-with-circles base-point p1 40.0 7)
    (fill-with-circles p1 p2 40.0 7)
    (make-c q 0.05 1 "Support")
    (make-c q 0.05 7 "Free")
    (alert "Push ENTER")
  )
)


; Delete layer
(defun animate (sloi / S) (setq S (ssget "X" (list (cons 8 sloi)))) (command ".ERASE" S "") (princ))

(defun link (ps pend / tita rad p1 p2 p3 p4 p5 p6)
  (setq tita (angle ps pend)
        rad 8.5
        B 20.0 ; shirina na zwenoto
        p1 (polar ps tita rad )
        p2 (polar pend tita (- rad) )
        p3 (polar ps (+ tita (/ pi 2)) B)
        p4 (polar ps (- tita (/ pi 2)) B)
        p5 (polar pend (+ tita (/ pi 2)) B)
        p6 (polar pend (- tita (/ pi 2)) B))
  (make-c ps rad 4 "Link")
  (make-c pend rad 4 "Link")
  (line1 p1 p2 "Link" 4)
  (setq list-of-points (list p3 p5 p6 p4))
  (make-lwpol list-of-points t)
)


; Draw line
(defun line1 (p1 p2 sloj col1 / ) (entmake (list '(0 . "LINE") (cons 8 sloj) (cons 62 col1) (cons 10 p1) (cons 11 p2))) (princ))


; Draw circle
(defun make-c (cen rad col layer1 / ) (entmake (list '(0 . "CIRCLE")  (cons 8 layer1) (cons 62 col) (cons 10 cen) (cons 40 rad))) (princ))


; Draw base
(defun opora (p1 rad / p2 p3 p4 p5)
  (setq  p2 (polar p1 (- 1.04719755) (* rad 6))
	 p3 (polar p1 4.1887902 (* rad 6))
	 p4 (polar p1 (- 1.04719755) rad)
	 p5 (polar p1 4.1887902 rad))
  (line1 p4 p2 "Support" 3)
  (line1 p5 p3 "Support" 3)
  (line1 p2 p3 "Support" 3)
  (make-c p1 rad 3 "Support")
  (princ)
)


; Fill the robotic arm with circles
(defun fill-with-circles (p1 p2 B n / a L R ang)
  (setq L (distance p1 p2)
	a0 0.0
	a (/ L (* 2.0 n))
	ang (angle p1 p2)
	R  (sqrt (+ (* (/ B 2.0) (/ B 2.0)) (* a a)))
  )
  (repeat (+ n 1)
    (make-c p1 R 6 "circle")
    (setq p1 (polar p1 ang (+ a0 (* a 2.0))))
  )
)


; Draw polyline
(defun make-lwpol (vertices closed)
  (entmake
    (append
      (list '(0 . "LWPOLYLINE") '(100 . "AcDbEntity") '(100 . "AcDbPolyline")
        (cons 8 "Link")
        (cons 62 1)
        (cons 90 (length vertices))
        (cons 70 (if closed 1 0)))
      (mapcar '(lambda (x) (cons 10 x)) vertices)
     )
  )
  (princ)
)


; Connect the points marking the path
(defun print-polyline-path (path / i)
  (if (> (length path) 2)
    (progn
      (setq i 1)
      (while (< i (length path))
	(setq first (forward-kinematics (nth 0 (nth (- i 1) path)) (nth 1 (nth (- i 1) path))))
	(setq second (forward-kinematics (nth 0 (nth i path)) (nth 1 (nth i path))))
	(line1 first second "path" 7)
	(setq i (+ i 1))
      )
    )
  )
)
