---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: splash
classes:
    - landing
    - dark-theme
    - wide
header:
    overlay_color: "#000"
    overlay_filter: "0.5"
    overlay_image: /assets/images/bg-3.jpg
author_profile: true

excerpt: "A blog about robotics, computer vision, deep learning, reinforcement learning and things I learn along the way."

feature_row:
    - image_path: /assets/images/Reinforcement-Learning/Blackjack-4.webp
      alt: "Reinforcement Learning"
      title: "Reinforcement Learning"
      excerpt: "Reinforcement learning is a machine learning technique that enables an agent to learn in an interactive environment by trial and error using feedback from its own actions and experiences. In this blog, I will provide a comprehensive introduction to reinforcement learning."
      url: "/categories/"
      btn_label: "Read More"
      btn_class: "btn--primary"
    - image_path: /assets/images/Nav2/local_cost_map.png
      alt: "Deep Learning"
      title: "Deep Learning"
      excerpt: "Deep learning is a subset of machine learning that uses neural networks to learn from data. In this blog, I will provide a comprehensive introduction to deep learning."
      url: "/categories/"
      btn_label: "Read More"
      btn_class: "btn--primary"
    - image_path: /assets/images/Nav2/global_planner.png
      alt: "Robotics"
      title: "Robotics"
      excerpt: "Robotics is an interdisciplinary field that integrates computer science and engineering. In this blog, I will provide a comprehensive introduction to robotics."
      url: "/categories/"
      btn_label: "Read More"
      btn_class: "btn--primary"

bio_image:
    - image_path: /assets/images/bio-photo-2.jpg
      alt: "profile picture"
      excerpt: "My name is Amit Nativ. I am a computer vision researcher specializing in robotics. Along my career I had the opprotunity to work on drones, autonomous vehicles and humanoids. I love what I do! This blog is a collection of my thoughts and notes on things I learn a long the way."
      # url: "/about/"
      btn_label: "Read More"
      btn_class: "btn--primary"


---


{% include feature_row id="bio_image" type="left"%}

{% include feature_row id="feature_row"%}