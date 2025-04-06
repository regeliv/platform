pub struct RunningAverage {
    n: usize,
    mean: f32,
}

impl RunningAverage {
    pub fn new() -> Self {
        RunningAverage { n: 0, mean: 0.0 }
    }

    pub fn update(&mut self, value: f32) {
        // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online
        self.n += 1;
        self.mean = self.mean + (value - self.mean) / (self.n as f32);
    }

    pub fn get(&self) -> f32 {
        self.mean
    }
}

pub struct RunningMin {
    min: Option<f32>,
}

impl RunningMin {
    pub fn new() -> Self {
        Self { min: None }
    }

    pub fn update(&mut self, value: f32) {
        match self.min {
            None => self.min = Some(value),
            Some(current_min) => {
                if value < current_min {
                    self.min = Some(value)
                }
            }
        }
    }

    pub fn get(&self) -> Option<f32> {
        self.min
    }
}
