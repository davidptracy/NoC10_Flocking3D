class Boid {

  PVector location, velocity, acceleration;  

  //  This stuff is now declared globally
  //  float radius, maxForce, maxSpeed;
  //  float desiredSeparation, alignThreshold, cohesionThreshold;

  //  This ArrayList is storing the location coordinates for the boids
  ArrayList<PVector> history = new ArrayList<PVector>();

  Boid(PVector l, float _radius) {
    acceleration = new PVector(0, 0, 0);
    velocity = new PVector(random(-5, 5), random(-5, 5), random(-5, 5));
    location = l;

    //    radius = _radius;
    //    maxForce = .05;
    //    maxSpeed = 1;
    //    desiredSeparation = 50.0;
    //    alignThreshold = 150.0;
    //    cohesionThreshold = 150.0;
  }

  // An ArrayList of boids is being passed to run() from the Flock class.
  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
    drawHistory();
  }

  void applyForce(PVector force) {
    acceleration.add(force);
  }

  // The boids ArrayList is coming from run()
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids, desiredSeparation); 
    PVector ali = align(boids, alignThreshold);
    PVector coh = cohesion(boids, cohesionThreshold);
    // Arbitrary weights for these forces
    sep.mult(sepVal);
    ali.mult(aliVal);
    coh.mult(cohVal);
    // Passes the adjusted forces to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }

  void update() {
    velocity.add(acceleration);
    // Limits the Velocity of each boid
    velocity.limit(maxSpeed);
    location.add(velocity);    
    //  Reset acceleration to 0 at each cycle
    acceleration.mult(0);
  }

  void drawHistory() {
    if (frameCount % 30 == 0) {
      history.add(location.get());
    }
    if (history.size() > 15) {
      for (int i=history.size(); i<history.size()-15; i--) {
        //get the last 15 locations from the array list
        PVector historyLoc = new PVector(history.get(i).x, history.get(i).y, history.get(i).z);      
        translate(history.get(i).x, history.get(i).y, history.get(i).z);
        pushMatrix();
        noStroke();
        box(20);
        popMatrix();
      }
    }
  }

  void render() {
    colorMode(HSB, 100);
    //float theta = velocity.heading() + radians(90);
    fill(map(dist(location.x, location.y, location.z, 0, 0, 0), 0, 800, 0, 100), 100, 100);
    radius = map(dist(location.x, location.y, location.z, 0, 0, 0), 0, 400, 25, 2);
    pushMatrix();
    translate(location.x, location.y, location.z);
    //rotate(theta);
    noStroke();    
    box(radius);
    popMatrix();
  }

  // This method makes the boids bounce off the edges
  void borders() {
    if (location.x - radius < -250) { 
      location.x = -250 + radius;
      velocity.x *= -1;
    }
    if (location.x + radius > 250) { 
      location.x = 250 - radius;
      velocity.x *= -1;
    }
    if (location.y - radius < -250) { 
      location.y = -250 + radius;
      velocity.y *= -1;
    }
    if (location.y + radius > 250) { 
      location.y = 250 - radius;
      velocity.y *= -1;
    }
    if (location.z - radius < -250) { 
      location.z = -250 + radius;
      velocity.z *= -1;
    }
    if (location.z + radius > 250) { 
      location.z = 250 - radius;
      velocity.z *= -1;
    }
  }


  // STEER ===================================================
  // This method calculates a steering force towards a target. (STEER = DESIRED - VELOCITY)
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);
    // Normalize desired and scale it to maximum speed
    desired.normalize();
    desired.mult(maxSpeed);
    PVector steer = PVector.sub(desired, velocity);
    // Limit to maximum steering force
    steer.limit(maxForce);
    return steer;
  }

  // SEPARATION ===================================================
  // This method checks for nearby boids and steers away from them.
  // 'desired' is initialized in the constructor and passed to separate from run()
  PVector separate(ArrayList<Boid> boids, float desired) {
    float desiredSep = desired;
    PVector steer = new PVector(0, 0, 0);
    // This integer is keeping track of how many neighbors are within the boids theshold.
    // It will then use this count to apply an average multiplier to the separation forces
    int count = 0;
    // Check every other boid in the system to see if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < desiredSep)) {
        // Calculate a vector that's pointing away from its its neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);
        steer.add(diff);
        count ++;
      }
    }
    // This is generating the average.
    if (count > 0) {
      steer.div(count);
    }
    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(maxSpeed);
      steer.sub(velocity);
      steer.limit(maxForce);
    }
    return steer;
  }

  // ALIGNMENT ======================================================
  // Alignment makes the boids align themselves to the same average heading
  PVector align(ArrayList<Boid> boids, float threshold) {
    float alignThresh = threshold;
    PVector sum = new PVector(0, 0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < alignThresh)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.normalize();
      sum.mult(maxSpeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxForce);
      return steer;
    } 
    else {
      return new PVector(0, 0, 0);
    }
  }

  // COHESION ========================================================
  // Cohesion makes the boids steer toward the average center point of their neighbors
  PVector cohesion(ArrayList<Boid> boids, float attraction) {
    float cohesionThresh = attraction;
    // Start with an empty vector to accumulate all locations
    PVector sum = new PVector(0, 0, 0); 
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d >0) && (d < cohesionThresh)) {
        sum.add(other.location);
        count++;
      }
    }
    if (count >0) {
      sum.div(count);
      return seek(sum);
    } 
    else {
      return new PVector(0, 0, 0);
    }
  }
}

