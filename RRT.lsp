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

; Define the robot arm kinematics
(defun forward-kinematics (theta1 theta2)
  (setq x 0.0  ; x-coordinate of end effector
        y 0.0) ; y-coordinate of end effector
  (setq x (+ (* l1 (cos theta1)) (* l2 (cos (+ theta1 theta2)))))
  (setq y (+ (* l1 (sin theta1)) (* l2 (sin (+ theta1 theta2)))))
  (list x y)
)

;(node q parent)
(defun rrt (q0 qgoal max-iterations delta hand / tree i continue qrand qnear qnew)
  ; Declare the variables used in the function
  (setq hand-restriction (list (- (/ pi 2)) (/ pi 2)))
  (if (= hand "right")
    (setq hand-restriction (list 0 (/ pi 2)))
  )
  (if (= hand "left")
    (setq hand-restriction (list (- (/ pi 2)) 0))
  )
  (setq tree (list (list q0 nil)))
  (setq goal-found nil)
  (setq i 0)
  (while (and (< i max-iterations) (not goal-found))
    (setq continue nil)
    (setq qrand (list (random-angle (nth 0 qmin) (nth 0 qmax))
                      (random-angle (nth 1 qmin) (nth 1 qmax))))
    (if (not (valid-configuration qrand hand-restriction))
        (progn
          (setq i (+ i 1)) ; Increment i
          (setq continue t)))
    (if (and (not continue))
      (progn
        (setq qnear (nearest-neighbor qrand tree))
        (setq qnew (new-configuration qnear qrand delta))
        (if (not (valid-configuration qnew hand-restriction))
            (progn
              (setq i (+ i 1)) ; Increment i
              (setq continue t)))
        (if (and (< (distance1 qnew qgoal) delta) (not continue))
            (progn (setq goal-found t)
                   (setq tree (append tree (list (list qnew qnear))))
                   (setq i (+ i 1)) ; Increment i
                   (setq continue t)))
        (if (and (not continue))
	  (progn
            (setq tree (append tree (list (list qnew qnear))))
            (setq i (+ i 1)) ; Increment i
	  )
        )
      )
    )
  )
  (print i)
  (print goal-found)
  (print max-iterations)
  (print (distance1 qnew qgoal))
  (if goal-found (append tree (list (list qgoal qnew))) nil)
)

; Define helper functions
(defun valid-configuration (q hand-restriction)
  (and (<= (nth 0 qmin) (nth 0 q) (nth 0 qmax))
       (<= (nth 1 qmin) (nth 1 q) (nth 1 qmax))
       (<= (nth 0 hand-restriction) (nth 1 q) (nth 1 hand-restriction))
       (notCollision q))
)

(defun nearest-neighbor (q tree)
  (setq nearest nil)
  (setq min-distance nil)
  (foreach node tree
    (setq qn (car node))
    (setq d (distance1 q qn))
    (if (or (null min-distance) (< d min-distance)) 
        (progn (setq nearest qn)
               (setq min-distance d))))
  nearest
)

(defun new-configuration (qnear qrand delta)
  (setq distance2 (distance1 qnear qrand))
  (if (< distance2 delta) qrand
    (progn
      (setq theta (atan (- (nth 1 qrand) (nth 1 qnear))
                        (- (nth 0 qrand) (nth 0 qnear))))
      (setq qnew (list (+ (nth 0 qnear) (* delta (cos theta)))
                       (+ (nth 1 qnear) (* delta (sin theta)))))
  qnew))
)

(defun notCollision (q)
  (setq p1 (polar base-point (nth 0 q) l1)
	p2 (polar p1 (+ (nth 0 q) (nth 1 q)) l2))
    
    (and (not (check-collisions base-point p1 40.0 7))
	 (not (check-collisions p1 p2 40.0 7)))
)

(defun check-collisions (p1 p2 B n / a L R ang)
  (setq L (distance p1 p2)
	a0 0.0
	a (/ L (* 2.0 n))
	ang (angle p1 p2)
	R  (sqrt (+ (* (/ B 2.0) (/ B 2.0)) (* a a)))
	colision nil
  )

  (repeat (+ n 1)
    (foreach obst obstacles
      (if (< (distance p1 (nth 0 obst)) (+ R (nth 1 obst))) (setq colision t))
    )
    (setq p1 (polar p1 ang (+ a0 (* a 2.0))))
  )
  colision
)

(defun distance1 (q1 q2)
  (setq p1 (forward-kinematics (nth 0 q1) (nth 1 q1))
	p2 (forward-kinematics (nth 0 q2) (nth 1 q2)))
  (setq dx (- (nth 0 p1) (nth 0 p2)))
  (setq dy (- (nth 1 p1) (nth 1 p2)))
  (sqrt (+ (* dx dx) (* dy dy)))
)

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

(defun random-angle (angle1 angle2)
  (+ angle1 (* (rnd) (- angle2 angle1)))
)

(defun extract-path (last-node tree)
  (if (null last-node)
    '()
    (progn
      (setq config (car last-node)
            parent (cadr last-node))
      (cons config (extract-path (assoc parent tree) tree))
    )
  )
)

; Run the RRT algorithm
(setq build-tree (rrt q0 goal 4000 50 "right"))
(setq path (reverse (extract-path (last build-tree) build-tree)))

; Draw path
(defun draw-robot-path (path / p1 p2 q obst)
  (opora base-point 8)
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
(defun line1 (p1 p2 sloj col1 / ) (entmake (list '(0 . "LINE") (cons 8 sloj) (cons 62 col1) (cons 10 p1) (cons 11 p2))) (princ) );end defun
; Draw circle
(defun make-c (cen rad col layer1 / ) (entmake (list '(0 . "CIRCLE")  (cons 8 layer1) (cons 62 col) (cons 10 cen) (cons 40 rad))) (princ)) ;end _ make-c

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