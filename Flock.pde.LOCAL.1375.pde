class Flock {

  ArrayList<Boid> boids; // An ArrayList for all the boids
  //  ArrayList<VBrush> brushes; // An ArrayList for all the boids

    Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  void run() {
    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }

    println(boids.get(0).history.size());
  }

  void addBoid(Boid b) {
    boids.add(b);
  }
}
